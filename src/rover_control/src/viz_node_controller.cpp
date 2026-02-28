// rover_viz_node.cpp
// ============================================================
// Subscribes to /Pivot_Drive and /Pivot_Rotate (Float64MultiArray)
// Publishes MarkerArray to /rover_viz for RViz2 display.
//
// Layout (flat HUD in RViz, viewed top-down at z=0):
//
//   [ DIRECTION ]    [ DRIVE BAR ]    [ MODE ]
//
// DIRECTION panel (x=-3):
//   Arrow rotates to mirror physical joystick position.
//   Pivot_Rotate: Right=+250, Left=-250, Up=0(FWD), Down=0(REV)
//   Left/right from val. Up/down inferred from FWD/REV state.
//
// DRIVE BAR panel (x=0):
//   Horizontal bar growing 0→1 showing trigger pressure.
//   Green when active, grey when idle.
//
// MODE panel (x=+3):
//   FWD/REV/IDLE detected from Pivot_Drive sign:
//     MotorCompiler FWD → positive output → is_reverse=false
//     MotorCompiler REV → negative output → is_reverse=true
// ============================================================

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cmath>
#include <string>

// Euler yaw → quaternion (z-axis rotation only)
struct Quat { double x, y, z, w; };
Quat yaw_to_quat(double yaw)
{
    return {0.0, 0.0, std::sin(yaw / 2.0), std::cos(yaw / 2.0)};
}

// ---------------------------------------------------------------
// Direction arrow angle
//
// Pivot_Rotate: Right=+250, Left=-250, Up=0(FWD), Down=0(REV)
// val encodes only the X component of the joystick.
// We reconstruct a 2D unit vector:
//   x = val / 250  (clamped -1..1)
//   y = sqrt(1-x^2) * (+1 if FWD, -1 if REV)
// Then yaw = atan2(y, x)
// This means:
//   val=+250  → x=+1, y=0 → yaw=0       (arrow right)
//   val=-250  → x=-1, y=0 → yaw=PI      (arrow left)
//   val=0 FWD → x=0,  y=+1→ yaw=PI/2   (arrow up)
//   val=0 REV → x=0,  y=-1→ yaw=-PI/2  (arrow down)
// ---------------------------------------------------------------
double direction_yaw(double rotate_val, bool is_reverse)
{
    double x = rotate_val / 250.0;
    x = std::max(-1.0, std::min(1.0, x));
    double y = std::sqrt(std::max(0.0, 1.0 - x * x)) * (is_reverse ? -1.0 : 1.0);
    return std::atan2(y, x);
}

std::string direction_label(double rotate_val, bool is_reverse)
{
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
        drive_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "Pivot_Drive", 10,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                if (!msg->data.empty()) last_drive_ = msg->data[0];
            });

        rotate_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "Pivot_Rotate", 10,
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                if (!msg->data.empty()) last_rotate_ = msg->data[0];
            });

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "rover_viz", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RoverVizNode::publish_markers, this));

        RCLCPP_INFO(this->get_logger(), "RoverVizNode started → publishing /rover_viz");
    }

private:
    void publish_markers()
    {
        visualization_msgs::msg::MarkerArray array;
        int id = 0;
        const std::string F = "base_link";
        auto now = this->now();
        auto lt  = rclcpp::Duration::from_seconds(0.5);

        using M = visualization_msgs::msg::Marker;

        // ---------------------------------------------------------------
        // Compute state
        // ---------------------------------------------------------------

        // FWD/REV from Pivot_Drive sign (MotorCompiler output):
        //   reverseOn=false → ScalingAlgorithm(motor, 0, +1, 1,-1) → positive when trigger pressed
        //   reverseOn=true  → ScalingAlgorithm(motor, 0, -1, 1,-1) → negative when trigger pressed
        bool is_reverse = (last_drive_ < -0.02);
        bool is_driving = (std::abs(last_drive_) > 0.02);
        double bar_fill = std::min(1.0, std::abs(last_drive_));

        double yaw      = direction_yaw(last_rotate_, is_reverse);
        std::string dir = direction_label(last_rotate_, is_reverse);

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

            // Rotating arrow
            {
                auto arrow = make_marker(F, now, id++, M::ARROW,
                    X, 0, 0.05,  0.75, 0.10, 0.10,  1.0f,0.85f,0.0f, 1.0f, lt);
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
                array.markers.push_back(make_text(F, now, id++, buf,
                    X, -1.35, 0.1,  0.22,  1.0f,0.85f,0.0f, lt));
            }
        }

        // ---------------------------------------------------------------
        // PANEL 2: DRIVE BAR  (centred at x = 0.0)
        // ---------------------------------------------------------------
        {
            const double X      = 0.0;
            const double BW     = 1.8;    // bar track full width
            const double BH     = 0.40;   // bar height
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

            // Filled portion (grows left→right)
            if (bar_fill > 0.01)
            {
                double fw  = bar_fill * BW;
                double fcx = B_LEFT + fw / 2.0;
                float gr = is_driving ? 0.1f : 0.3f;
                float gg = is_driving ? 1.0f : 0.3f;
                float gb = is_driving ? 0.2f : 0.3f;
                array.markers.push_back(make_marker(F, now, id++, M::CUBE,
                    fcx, 0, 0.03,  fw, BH, 0.04,  gr, gg, gb, 1.0f, lt));
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
                snprintf(buf, sizeof(buf), "%.2f", last_drive_);
                array.markers.push_back(make_text(F, now, id++, buf,
                    X, -1.35, 0.1,  0.22,  0.7f,0.7f,0.7f, lt));
            }
        }

        // ---------------------------------------------------------------
        // PANEL 3: MODE (FWD / REV / IDLE)  (centred at x = +3.0)
        // ---------------------------------------------------------------
        {
            const double X = 3.0;

            // Background plate
            array.markers.push_back(make_marker(F, now, id++, M::CUBE,
                X, 0, -0.05,  2.4, 2.4, 0.05,  0.15f,0.15f,0.20f, 0.9f, lt));

            // Title
            array.markers.push_back(make_text(F, now, id++, "MODE",
                X, 1.35, 0.1,  0.25,  0.9f,0.9f,0.9f, lt));

            // Indicator cylinder — colour reflects state
            float cr, cg, cb;
            if (!is_driving)       { cr=0.30f; cg=0.30f; cb=0.30f; }  // grey  = idle
            else if (is_reverse)   { cr=1.00f; cg=0.15f; cb=0.15f; }  // red   = reverse
            else                   { cr=0.10f; cg=0.90f; cb=0.20f; }  // green = forward

            array.markers.push_back(make_marker(F, now, id++, M::CYLINDER,
                X, 0, 0.05,  1.3, 1.3, 0.1,  cr, cg, cb, 1.0f, lt));

            // Status text on cylinder
            std::string status = !is_driving ? "IDLE" : (is_reverse ? "REV" : "FWD");
            array.markers.push_back(make_text(F, now, id++, status,
                X, 0, 0.15,  0.30,  1.0f,1.0f,1.0f, lt));

            // Raw Pivot_Drive value below panel
            {
                char buf[32];
                snprintf(buf, sizeof(buf), "drive: %.2f", last_drive_);
                array.markers.push_back(make_text(F, now, id++, buf,
                    X, -1.35, 0.1,  0.20,  0.7f,0.7f,0.7f, lt));
            }
        }

        // ---------------------------------------------------------------
        marker_pub_->publish(array);
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr drive_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rotate_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double last_drive_  = 0.0;
    double last_rotate_ = 0.0;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoverVizNode>());
    rclcpp::shutdown();
    return 0;
}