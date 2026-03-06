// rover_viz_node.cpp — RViz HUD for /Pivot_Drive and /Pivot_Rotate
//
// Pivot_Rotate truth table (echo-verified):
//   RIGHT     +250  false     UP-RIGHT  +125  false
//   UP           0  false     UP-LEFT   -125  false
//   LEFT      -250  false     DOWN-LEFT +125  true
//   DOWN         0  true      DOWN-RIGHT-125  true
//
// val   = horizontal lean within each half (-250=leftmost, +250=rightmost)
// reverseOn = false → upper half,  true → lower half (val sign mirrored in lower half)
//
// reverseOn is not published; inferred from Pivot_Drive sign when trigger active,
// persisted in last_reverse_on_ when trigger idle.

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cmath>
#include <string>
#include <vector>

// ---------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------
struct Quat { double x, y, z, w; };
Quat yaw_to_quat(double yaw)
{
    return {0.0, 0.0, std::sin(yaw / 2.0), std::cos(yaw / 2.0)};
}

double val_to_display_yaw(double val, bool reverse_on)
{
    double x = std::max(-1.0, std::min(1.0, val / 250.0));
    if (reverse_on) x = -x;   // lower half has mirrored val sign
    double y = std::sqrt(std::max(0.0, 1.0 - x * x)) * (reverse_on ? -1.0 : 1.0);
    return std::atan2(y, x);
}

std::string yaw_to_label(double yaw)
{
    double deg = yaw * 180.0 / M_PI;
    while (deg >  180.0) deg -= 360.0;
    while (deg <= -180.0) deg += 360.0;
    double a = std::abs(deg);
    if (a < 22.5)              return "RIGHT";
    if (a > 157.5)             return "LEFT";
    if (deg > 0 && a < 67.5)  return "UP-RIGHT";
    if (deg > 0 && a < 112.5) return "UP";
    if (deg > 0)               return "UP-LEFT";
    if (deg < 0 && a < 67.5)  return "DOWN-RIGHT";
    if (deg < 0 && a < 112.5) return "DOWN";
    return                            "DOWN-LEFT";
}

// ---------------------------------------------------------------
// Marker helpers
// ---------------------------------------------------------------
using MArray = visualization_msgs::msg::MarkerArray;
using Marker = visualization_msgs::msg::Marker;

Marker make_marker(
    const std::string& frame, const rclcpp::Time& now, int id, int type,
    double px, double py, double pz,
    double sx, double sy, double sz,
    float r, float g, float b, float a,
    const rclcpp::Duration& lt)
{
    Marker m;
    m.header.frame_id    = frame;
    m.header.stamp       = now;
    m.ns                 = "rover_viz";
    m.id                 = id;
    m.type               = type;
    m.action             = Marker::ADD;
    m.pose.position.x    = px;
    m.pose.position.y    = py;
    m.pose.position.z    = pz;
    m.pose.orientation.w = 1.0;
    m.scale.x = sx; m.scale.y = sy; m.scale.z = sz;
    m.color.r = r;  m.color.g = g;  m.color.b = b;  m.color.a = a;
    m.lifetime = lt;
    return m;
}

Marker make_text(
    const std::string& frame, const rclcpp::Time& now, int id,
    const std::string& text,
    double px, double py, double pz,
    double size, float r, float g, float b,
    const rclcpp::Duration& lt)
{
    auto m = make_marker(frame, now, id, Marker::TEXT_VIEW_FACING,
        px, py, pz, 0, 0, size, r, g, b, 1.0f, lt);
    m.text = text;
    return m;
}

// ---------------------------------------------------------------
// Node
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

        // Subscribe to /joy directly to get joystick Y axis for reverse detection.
        // axes[1] = left joystick Y: positive = up, negative = down (lower half).
        // axes[7] = d-pad Y: same convention.
        // This lets the arrow show correct half even when trigger is not pressed.
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
                double joy_y  = (msg->axes.size() > 1) ? msg->axes[1] : 0.0;
                double dpad_y = (msg->axes.size() > 7) ? msg->axes[7] : 0.0;
                double y = (joy_y != 0.0) ? joy_y : dpad_y;
                last_reverse_on_ = (y < -0.1);   // negative Y = lower half = reverseOn
            });

        marker_pub_ = this->create_publisher<MArray>("rover_viz", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RoverVizNode::publish_markers, this));

        RCLCPP_INFO(this->get_logger(), "RoverVizNode started → /rover_viz");
    }

private:
    void publish_markers()
    {
        MArray array;
        int id = 0;
        const std::string F = "base_link";
        auto now = this->now();
        auto lt  = rclcpp::Duration::from_seconds(0.5);

        // ---- State ----
        bool is_driving = (std::abs(last_drive_) > 0.02);
        bool reverse_on      = last_reverse_on_;   // set directly by /joy subscription
        bool is_reverse_mode = (is_driving && last_drive_ < 0.0);
        double bar_fill      = std::min(1.0, std::abs(last_drive_));

        double yaw      = val_to_display_yaw(last_rotate_, reverse_on);
        std::string dir = yaw_to_label(yaw);

        // ================================================================
        // PANEL 1 — DIRECTION  (centred x = -3)
        // ================================================================
        {
            const double X = -3.0;

            // Background plate
            array.markers.push_back(make_marker(F, now, id++, Marker::CUBE,
                X, 0, -0.05,  2.4, 2.4, 0.05,
                0.15f, 0.15f, 0.20f, 0.9f, lt));

            // Title
            array.markers.push_back(make_text(F, now, id++, "DIRECTION",
                X, 1.35, 0.1,  0.25,  0.9f, 0.9f, 0.9f, lt));

            // Ring labels
            struct RL { const char* t; double dx, dy; };
            for (auto& rl : std::vector<RL>{
                    {"UP",    0.0,  0.85},
                    {"DOWN",  0.0, -0.85},
                    {"LEFT", -0.85, 0.0 },
                    {"RIGHT", 0.85, 0.0 }})
            {
                array.markers.push_back(make_text(F, now, id++, rl.t,
                    X + rl.dx, rl.dy, 0.1,  0.17,  0.5f, 0.5f, 0.5f, lt));
            }

            // Rotating arrow
            {
                auto arrow = make_marker(F, now, id++, Marker::ARROW,
                    X, 0, 0.05,  0.75, 0.10, 0.10,
                    1.0f, 0.85f, 0.0f, 1.0f, lt);
                auto q = yaw_to_quat(yaw);
                arrow.pose.orientation.x = q.x;
                arrow.pose.orientation.y = q.y;
                arrow.pose.orientation.z = q.z;
                arrow.pose.orientation.w = q.w;
                array.markers.push_back(arrow);
            }

            // Direction label + raw value
            {
                char buf[48];
                snprintf(buf, sizeof(buf), "%s\n(%.0f)", dir.c_str(), last_rotate_);
                array.markers.push_back(make_text(F, now, id++, buf,
                    X, -1.35, 0.1,  0.22,  1.0f, 0.85f, 0.0f, lt));
            }
        }

        // ================================================================
        // PANEL 2 — DRIVE BAR  (centred x = 0)
        // ================================================================
        {
            const double X      = 0.0;
            const double BW     = 1.8;
            const double BH     = 0.40;
            const double B_LEFT = X - BW / 2.0;

            // Background plate
            array.markers.push_back(make_marker(F, now, id++, Marker::CUBE,
                X, 0, -0.05,  2.4, 2.4, 0.05,
                0.15f, 0.15f, 0.20f, 0.9f, lt));

            // Title
            array.markers.push_back(make_text(F, now, id++, "DRIVE",
                X, 1.35, 0.1,  0.25,  0.9f, 0.9f, 0.9f, lt));

            // Bar track
            array.markers.push_back(make_marker(F, now, id++, Marker::CUBE,
                X, 0, 0.0,  BW + 0.07, BH + 0.07, 0.04,
                0.08f, 0.08f, 0.08f, 1.0f, lt));

            // Bar fill (grows left → right)
            if (bar_fill > 0.01) {
                double fw  = bar_fill * BW;
                double fcx = B_LEFT + fw / 2.0;
                float  gr  = is_driving ? 0.1f : 0.3f;
                float  gg  = is_driving ? 1.0f : 0.3f;
                float  gb  = is_driving ? 0.2f : 0.3f;
                array.markers.push_back(make_marker(F, now, id++, Marker::CUBE,
                    fcx, 0, 0.03,  fw, BH, 0.04,  gr, gg, gb, 1.0f, lt));
            }

            // Tick marks at 25 / 50 / 75 %
            for (double t : {0.25, 0.50, 0.75}) {
                double tx = B_LEFT + t * BW;
                array.markers.push_back(make_marker(F, now, id++, Marker::CUBE,
                    tx, 0, 0.05,  0.025, BH * 0.65, 0.05,
                    0.55f, 0.55f, 0.55f, 1.0f, lt));
            }

            // End labels
            array.markers.push_back(make_text(F, now, id++, "0",
                B_LEFT - 0.14, 0, 0.1,  0.18,  0.6f, 0.6f, 0.6f, lt));
            array.markers.push_back(make_text(F, now, id++, "1",
                B_LEFT + BW + 0.14, 0, 0.1,  0.18,  0.6f, 0.6f, 0.6f, lt));

            // Numeric value
            {
                char buf[32];
                snprintf(buf, sizeof(buf), "%.2f", last_drive_);
                array.markers.push_back(make_text(F, now, id++, buf,
                    X, -1.35, 0.1,  0.22,  0.7f, 0.7f, 0.7f, lt));
            }
        }

        // ================================================================
        // PANEL 3 — MODE: FWD / REV / IDLE  (centred x = +3)
        // ================================================================
        {
            const double X = 3.0;

            // Background plate
            array.markers.push_back(make_marker(F, now, id++, Marker::CUBE,
                X, 0, -0.05,  2.4, 2.4, 0.05,
                0.15f, 0.15f, 0.20f, 0.9f, lt));

            // Title
            array.markers.push_back(make_text(F, now, id++, "MODE",
                X, 1.35, 0.1,  0.25,  0.9f, 0.9f, 0.9f, lt));

            // Indicator cylinder
            float cr, cg, cb;
            if      (!is_driving)      { cr=0.30f; cg=0.30f; cb=0.30f; } // grey  = IDLE
            else if (is_reverse_mode)  { cr=1.00f; cg=0.15f; cb=0.15f; } // red   = REV
            else                       { cr=0.10f; cg=0.90f; cb=0.20f; } // green = FWD

            array.markers.push_back(make_marker(F, now, id++, Marker::CYLINDER,
                X, 0, 0.05,  1.3, 1.3, 0.1,  cr, cg, cb, 1.0f, lt));

            // Status text
            std::string status = !is_driving ? "IDLE" : (is_reverse_mode ? "REV" : "FWD");
            array.markers.push_back(make_text(F, now, id++, status,
                X, 0, 0.15,  0.30,  1.0f, 1.0f, 1.0f, lt));

            // Raw drive value
            {
                char buf[32];
                snprintf(buf, sizeof(buf), "drive: %.2f", last_drive_);
                array.markers.push_back(make_text(F, now, id++, buf,
                    X, -1.35, 0.1,  0.20,  0.7f, 0.7f, 0.7f, lt));
            }
        }

        marker_pub_->publish(array);
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr drive_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rotate_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<MArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double last_drive_      = 0.0;
    double last_rotate_     = 0.0;
    bool   last_reverse_on_ = false;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoverVizNode>());
    rclcpp::shutdown();
    return 0;
}
