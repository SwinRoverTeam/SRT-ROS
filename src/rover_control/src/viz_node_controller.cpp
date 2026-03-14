// rover_viz_node.cpp
// ============================================================
// Subscribes to /Pivot_Drive, /Pivot_Rotate, and /Pivot_Home.
// Publishes MarkerArray to /rover_viz for RViz2 display.
// No new publishers are added to the controller — all state is
// derived purely from the three existing topics.
//
// Layout (flat HUD in RViz, viewed top-down at z=0):
//
//   [ DIRECTION ]    [ DRIVE BAR ]    [ MODE ]
//
// DIRECTION panel (x=-3):
//   Arrow rotates to mirror physical wheel orientation.
//   Pivot_Rotate carries the wheel-angle value (val) from
//   JoystickAlgorithm.  Normal steering: val in [-250, +250].
//   Pivot (B/X): value is ±151 per wheel — detected separately
//   and shown as a PIVOT_RIGHT / PIVOT_LEFT label with a fixed
//   sideways arrow rather than a misleading angled one.
//
// DRIVE BAR panel (x=0):
//   Horizontal bar growing 0→1 showing trigger pressure.
//   Colour: green=FWD, red=REV, grey=idle.
//
// MODE panel (x=+3):
//   Reflects the DriveMode from MotorCompiler:
//     FWD        — wheels forward, trigger pressed forward
//     REV        — wheels reversed (reverseOn=true)
//     PIVOT_RIGHT — B-button pivot active
//     PIVOT_LEFT  — X-button pivot active
//     HOMING     — Menu button sent Pivot_Home=true
//     IDLE       — trigger at rest
//
// reverseOn detection:
//   MotorCompiler publishes via ScalingAlgorithm:
//     reverseOn=false → mtr_forward = ScalingAlgorithm(motor, 0, +1, 1,-1)
//                       At trigger rest (axes[4]=-1): mtr_forward = +1
//     reverseOn=true  → mtr_reverse = ScalingAlgorithm(motor, 0, -1, 1,-1)
//                       At trigger rest (axes[4]=-1): mtr_reverse = -1
//   So last_drive_[0] sign reliably encodes reverseOn at all times,
//   even when the trigger is not being pressed.
//
// Pivot detection:
//   When B or X is pressed, Pivot_Rotate[0] = ±151 (outside the
//   normal [-250,+250] steering range).  We use |val| > 140 as the
//   pivot sentinel.  The drive pattern alternates signs:
//     PIVOT_RIGHT: {mtr_reverse, mtr_forward, ...} → last_drive_[0]<0
//     PIVOT_LEFT:  {mtr_forward, mtr_reverse, ...} → last_drive_[0]>0
//   Combined with the pivot sentinel this uniquely identifies each.
// ============================================================

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cmath>
#include <string>

// ---------------------------------------------------------------
// Rover display state — derived entirely from the three topics
// ---------------------------------------------------------------
enum class VizMode {
    IDLE,
    FWD,
    REV,
    PIVOT_RIGHT,
    PIVOT_LEFT,
    HOMING
};

// Euler yaw → quaternion (z-axis rotation only)
struct Quat { double x, y, z, w; };
Quat yaw_to_quat(double yaw)
{
    return {0.0, 0.0, std::sin(yaw / 2.0), std::cos(yaw / 2.0)};
}

// ---------------------------------------------------------------
// Direction arrow angle
//
// Normal steering: val in [-250, +250]
//   x = val/250, y = sqrt(1-x^2) * (+1 FWD / -1 REV)
//   yaw = atan2(y, x)
//
// Pivot sentinel (|val| > 140): arrow locked sideways.
//   PIVOT_RIGHT → yaw = 0        (pointing right)
//   PIVOT_LEFT  → yaw = M_PI     (pointing left)
// ---------------------------------------------------------------
double direction_yaw(double rotate_val, bool is_reverse, bool is_pivot_right, bool is_pivot_left)
{
    if (is_pivot_right) return 0.0;
    if (is_pivot_left)  return M_PI;

    double x = rotate_val / 250.0;
    x = std::max(-1.0, std::min(1.0, x));
    double y = std::sqrt(std::max(0.0, 1.0 - x * x)) * (is_reverse ? -1.0 : 1.0);
    return std::atan2(y, x);
}

std::string direction_label(double rotate_val, bool is_reverse, bool is_pivot_right, bool is_pivot_left)
{
    if (is_pivot_right) return "PIVOT\nRIGHT";
    if (is_pivot_left)  return "PIVOT\nLEFT";

    double x = rotate_val / 250.0;
    x = std::max(-1.0, std::min(1.0, x));
    double abs_x = std::abs(x);

    if (abs_x < 0.2) {
        return is_reverse ? "DOWN" : "UP";
    } else if (abs_x > 0.85) {
        return x > 0 ? "RIGHT" : "LEFT";
    } else {
        std::string horiz = x > 0 ? "RIGHT" : "LEFT";
        std::string vert  = is_reverse ? "DOWN" : "UP";
        return vert + "-" + horiz;
    }
}

// ---------------------------------------------------------------
// Marker helpers — reduce boilerplate
// ---------------------------------------------------------------
visualization_msgs::msg::Marker make_marker(
    const std::string& frame, const rclcpp::Time& now,
    int id, int type,
    double px, double py, double pz,
    double sx, double sy, double sz,
    float r, float g, float b, float a,
    const rclcpp::Duration& lt)
{
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame;
    m.header.stamp    = now;
    m.ns              = "rover_viz";
    m.id              = id;
    m.type            = type;
    m.action          = visualization_msgs::msg::Marker::ADD;
    m.pose.position.x = px;
    m.pose.position.y = py;
    m.pose.position.z = pz;
    m.pose.orientation.w = 1.0;
    m.scale.x = sx; m.scale.y = sy; m.scale.z = sz;
    m.color.r = r;  m.color.g = g;  m.color.b = b;  m.color.a = a;
    m.lifetime = lt;
    return m;
}

visualization_msgs::msg::Marker make_text(
    const std::string& frame, const rclcpp::Time& now,
    int id, const std::string& text,
    double px, double py, double pz,
    double size, float r, float g, float b,
    const rclcpp::Duration& lt)
{
    auto m = make_marker(frame, now, id,
        visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
        px, py, pz, 0, 0, size, r, g, b, 1.0f, lt);
    m.text = text;
    return m;
}

// ---------------------------------------------------------------
class RoverVizNode : public rclcpp::Node
{
public:
    RoverVizNode() : Node("rover_viz_node")
    {
        // Pivot_Drive — all four motor commands published by MotorCompiler.
        // We store all four values so we can detect the alternating pattern
        // that identifies PIVOT_RIGHT vs PIVOT_LEFT.
        drive_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "Pivot_Drive", 10,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                if (msg->data.size() >= 4) {
                    last_drive_ = msg->data;  // store full array
                }
            });

        // Pivot_Rotate — wheel-angle targets from JoystickAlgorithm.
        // |val| > 140 is the pivot sentinel (normal range is [-250, +250]).
        rotate_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "Pivot_Rotate", 10,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                if (!msg->data.empty()) last_rotate_ = msg->data[0];
            });

        // Pivot_Home — one-shot bool published by Menu button.
        // Latch it for HOMING_DISPLAY_MS so it remains visible in RViz.
        home_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "Pivot_Home", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data) {
                    homing_active_    = true;
                    homing_latch_end_ = this->now() + rclcpp::Duration::from_seconds(HOMING_DISPLAY_S);
                }
            });

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "rover_viz", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RoverVizNode::publish_markers, this));

        RCLCPP_INFO(this->get_logger(), "RoverVizNode started → publishing /rover_viz");
    }

private:
    // How long the HOMING label stays visible after the one-shot pulse
    static constexpr double HOMING_DISPLAY_S = 2.0;

    // ---------------------------------------------------------------
    // Derive VizMode from the three topic values
    // ---------------------------------------------------------------
    VizMode resolve_mode(bool is_pivot_right, bool is_pivot_left) const
    {
        // Homing takes priority — it's a one-shot event
        if (homing_active_) return VizMode::HOMING;

        if (is_pivot_right) return VizMode::PIVOT_RIGHT;
        if (is_pivot_left)  return VizMode::PIVOT_LEFT;

        // FWD/REV from sign of first drive value.
        // MotorCompiler ScalingAlgorithm at trigger rest (axes[4]=-1):
        //   reverseOn=false → mtr_forward = +1
        //   reverseOn=true  → mtr_reverse = -1
        // So the sign persists even when trigger is not pressed.
        double d0 = last_drive_.empty() ? 0.0 : last_drive_[0];
        bool is_driving = (std::abs(d0) > 0.02);
        bool is_reverse = (d0 < -0.02);

        if (!is_driving) return VizMode::IDLE;
        return is_reverse ? VizMode::REV : VizMode::FWD;
    }

    void publish_markers()
    {
        auto now = this->now();

        // Expire homing latch
        if (homing_active_ && now >= homing_latch_end_) {
            homing_active_ = false;
        }

        visualization_msgs::msg::MarkerArray array;
        int id = 0;
        const std::string F = "base_link";
        auto lt = rclcpp::Duration::from_seconds(0.5);

        using M = visualization_msgs::msg::Marker;

        // ---------------------------------------------------------------
        // Derive state from topics
        // ---------------------------------------------------------------
        double d0       = last_drive_.empty() ? 0.0 : last_drive_[0];
        bool is_reverse = (d0 < -0.02);
        bool is_driving = (std::abs(d0) > 0.02);
        double bar_fill = last_drive_.empty() ? 0.0 : std::min(1.0, std::abs(d0));

        // Pivot sentinel: Pivot_Rotate publishes {-151,151,151,-151} for B/X.
        // |val| > 140 is outside normal steering range → pivot active.
        // Drive[0] sign distinguishes PIVOT_RIGHT (negative) vs PIVOT_LEFT (positive).
        bool pivot_active = (std::abs(last_rotate_) > 140.0);
        bool is_pivot_right = pivot_active && (d0 < -0.02);
        bool is_pivot_left  = pivot_active && (d0 > 0.02);

        VizMode mode = resolve_mode(is_pivot_right, is_pivot_left);

        double yaw      = direction_yaw(last_rotate_, is_reverse, is_pivot_right, is_pivot_left);
        std::string dir = direction_label(last_rotate_, is_reverse, is_pivot_right, is_pivot_left);

        // ---------------------------------------------------------------
        // PANEL 1: DIRECTION  (centred at x = -3.0)
        // ---------------------------------------------------------------
        {
            const double X = -3.0;

            // Background plate
            array.markers.push_back(make_marker(F, now, id++, M::CUBE,
                X, 0, -0.05,  2.4, 2.4, 0.05,  0.15f,0.15f,0.20f, 0.9f, lt));

            // Title
            array.markers.push_back(make_text(F, now, id++, "DIRECTION",
                X, 1.35, 0.1,  0.25,  0.9f,0.9f,0.9f, lt));

            // Ring position labels
            struct RL { std::string t; double dx, dy; };
            for (auto& rl : std::vector<RL>{
                    {"UP",    0.0,  0.85},
                    {"DOWN",  0.0, -0.85},
                    {"LEFT", -0.85, 0.0},
                    {"RIGHT", 0.85, 0.0}})
            {
                array.markers.push_back(make_text(F, now, id++, rl.t,
                    X + rl.dx, rl.dy, 0.1,  0.17,  0.5f,0.5f,0.5f, lt));
            }

            // Rotating arrow — orange during normal steering, cyan during pivot
            {
                float ar = is_pivot_right || is_pivot_left ? 0.0f : 1.0f;
                float ag = is_pivot_right || is_pivot_left ? 0.9f : 0.85f;
                float ab = is_pivot_right || is_pivot_left ? 1.0f : 0.0f;
                auto arrow = make_marker(F, now, id++, M::ARROW,
                    X, 0, 0.05,  0.75, 0.10, 0.10,  ar, ag, ab, 1.0f, lt);
                auto q = yaw_to_quat(yaw);
                arrow.pose.orientation.x = q.x;
                arrow.pose.orientation.y = q.y;
                arrow.pose.orientation.z = q.z;
                arrow.pose.orientation.w = q.w;
                array.markers.push_back(arrow);
            }

            // Label + raw value below panel
            {
                char buf[48];
                snprintf(buf, sizeof(buf), "%s\n(%.0f)", dir.c_str(), last_rotate_);
                bool pivot = is_pivot_right || is_pivot_left;
                array.markers.push_back(make_text(F, now, id++, buf,
                    X, -1.35, 0.1,  0.22,
                    pivot ? 0.0f : 1.0f,
                    pivot ? 0.9f : 0.85f,
                    pivot ? 1.0f : 0.0f, lt));
            }
        }

        // ---------------------------------------------------------------
        // PANEL 2: DRIVE BAR  (centred at x = 0.0)
        // ---------------------------------------------------------------
        {
            const double X      = 0.0;
            const double BW     = 1.8;
            const double BH     = 0.40;
            const double B_LEFT = X - BW / 2.0;

            // Background plate
            array.markers.push_back(make_marker(F, now, id++, M::CUBE,
                X, 0, -0.05,  2.4, 2.4, 0.05,  0.15f,0.15f,0.20f, 0.9f, lt));

            // Title
            array.markers.push_back(make_text(F, now, id++, "DRIVE",
                X, 1.35, 0.1,  0.25,  0.9f,0.9f,0.9f, lt));

            // Bar track (dark trough)
            array.markers.push_back(make_marker(F, now, id++, M::CUBE,
                X, 0, 0.0,  BW + 0.07, BH + 0.07, 0.04,
                0.08f,0.08f,0.08f, 1.0f, lt));

            // Filled portion — green=FWD, red=REV, grey=idle
            if (bar_fill > 0.01)
            {
                double fw  = bar_fill * BW;
                double fcx = B_LEFT + fw / 2.0;
                float br, bg, bb;
                if (!is_driving)     { br=0.3f; bg=0.3f; bb=0.3f; }  // grey
                else if (is_reverse) { br=1.0f; bg=0.1f; bb=0.1f; }  // red
                else                 { br=0.1f; bg=1.0f; bb=0.2f; }  // green
                array.markers.push_back(make_marker(F, now, id++, M::CUBE,
                    fcx, 0, 0.03,  fw, BH, 0.04,  br, bg, bb, 1.0f, lt));
            }

            // Tick marks at 25 / 50 / 75 %
            for (double t : {0.25, 0.50, 0.75}) {
                double tx = B_LEFT + t * BW;
                array.markers.push_back(make_marker(F, now, id++, M::CUBE,
                    tx, 0, 0.05,  0.025, BH * 0.65, 0.05,
                    0.55f,0.55f,0.55f, 1.0f, lt));
            }

            // End labels "0" and "1"
            array.markers.push_back(make_text(F, now, id++, "0",
                B_LEFT - 0.14, 0, 0.1,  0.18,  0.6f,0.6f,0.6f, lt));
            array.markers.push_back(make_text(F, now, id++, "1",
                B_LEFT + BW + 0.14, 0, 0.1,  0.18,  0.6f,0.6f,0.6f, lt));

            // Numeric value below panel
            {
                char buf[32];
                snprintf(buf, sizeof(buf), "%.2f", d0);
                array.markers.push_back(make_text(F, now, id++, buf,
                    X, -1.35, 0.1,  0.22,  0.7f,0.7f,0.7f, lt));
            }
        }

        // ---------------------------------------------------------------
        // PANEL 3: MODE  (centred at x = +3.0)
        // ---------------------------------------------------------------
        {
            const double X = 3.0;

            // Background plate
            array.markers.push_back(make_marker(F, now, id++, M::CUBE,
                X, 0, -0.05,  2.4, 2.4, 0.05,  0.15f,0.15f,0.20f, 0.9f, lt));

            // Title
            array.markers.push_back(make_text(F, now, id++, "MODE",
                X, 1.35, 0.1,  0.25,  0.9f,0.9f,0.9f, lt));

            // Indicator cylinder colour and label — one case per VizMode
            float cr, cg, cb;
            std::string status;

            switch (mode) {
                case VizMode::FWD:
                    cr=0.10f; cg=0.90f; cb=0.20f;  // green
                    status = "FWD";
                    break;
                case VizMode::REV:
                    cr=1.00f; cg=0.15f; cb=0.15f;  // red
                    status = "REV";
                    break;
                case VizMode::PIVOT_RIGHT:
                    cr=0.00f; cg=0.60f; cb=1.00f;  // cyan-blue
                    status = "PIVOT\nRIGHT";
                    break;
                case VizMode::PIVOT_LEFT:
                    cr=0.00f; cg=0.60f; cb=1.00f;  // cyan-blue
                    status = "PIVOT\nLEFT";
                    break;
                case VizMode::HOMING:
                    cr=1.00f; cg=0.70f; cb=0.00f;  // amber
                    status = "HOMING";
                    break;
                case VizMode::IDLE:
                default:
                    cr=0.30f; cg=0.30f; cb=0.30f;  // grey
                    status = "IDLE";
                    break;
            }

            array.markers.push_back(make_marker(F, now, id++, M::CYLINDER,
                X, 0, 0.05,  1.3, 1.3, 0.1,  cr, cg, cb, 1.0f, lt));

            array.markers.push_back(make_text(F, now, id++, status,
                X, 0, 0.15,  0.25,  1.0f,1.0f,1.0f, lt));

            // Raw Pivot_Drive value below panel
            {
                char buf[32];
                snprintf(buf, sizeof(buf), "drive: %.2f", d0);
                array.markers.push_back(make_text(F, now, id++, buf,
                    X, -1.35, 0.1,  0.20,  0.7f,0.7f,0.7f, lt));
            }
        }

        // ---------------------------------------------------------------
        marker_pub_->publish(array);
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr drive_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rotate_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr              home_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<double> last_drive_;           // full 4-element Pivot_Drive array
    double              last_rotate_  = 0.0;   // Pivot_Rotate[0]
    bool                homing_active_    = false;
    rclcpp::Time        homing_latch_end_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoverVizNode>());
    rclcpp::shutdown();
    return 0;
}