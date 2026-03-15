// viz_node_controller.cpp
// RViz HUD for rover ground control.
//
// Subscribes to:
//   /Pivot_Drive      (Float64MultiArray) — drive magnitude
//   /Pivot_Rotate     (Float64MultiArray) — wheel angle value -250..+250
//   /Reverse_State    (Bool)              — reverseOn flag from JoystickAlgorithm
//   /Input_Mode       (Int32)             — InputMode enum from joystick_callback
//   /Drive_Mode       (Int32)             — DriveMode enum from MotorCompiler
//
// InputMode: NONE=0  JOYSTICK=1  GAMEPAD=2  TURN_RIGHT=3  TURN_LEFT=4  PIVOT_HOME=5
// DriveMode: FORWARD=0  REVERSE=1  PIVOT_RIGHT=2  PIVOT_LEFT=3
//
// Layout (top-down view in RViz, x-axis = left/right, y-axis = up/down):
//
//   x=-6   x=-3    x=0     x=3     x=6
//
//   [LB]  [DIRECTION]  [DRIVE BAR]  [MODE]
//
//   Button row below (y = -2.5):
//   [LB]   [B btn]   [X btn]   [Menu btn]

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cmath>
#include <string>
#include <vector>

// ---------------------------------------------------------------
// InputMode / DriveMode mirrors from controller
// ---------------------------------------------------------------
enum class InputMode { NONE=0, JOYSTICK=1, GAMEPAD=2, TURN_RIGHT=3, TURN_LEFT=4, PIVOT_HOME=5 };
enum class DriveMode { FORWARD=0, REVERSE=1, PIVOT_RIGHT=2, PIVOT_LEFT=3 };

// ---------------------------------------------------------------
// Math helpers
// ---------------------------------------------------------------
struct Quat { double x, y, z, w; };
Quat yaw_to_quat(double yaw)
{
    return {0.0, 0.0, std::sin(yaw/2.0), std::cos(yaw/2.0)};
}

// Pivot_Rotate truth table (echo-verified):
//   RIGHT +250/false  UP-RIGHT +125/false  UP-LEFT -125/false  UP    0/false
//   LEFT -250/false   DOWN-LEFT+125/true   DOWN-RIGHT-125/true DOWN   0/true
// val = horizontal lean, reverseOn = lower half flag, lower half val sign is mirrored.
double val_to_display_yaw(double val, bool reverse_on)
{
    double x = std::max(-1.0, std::min(1.0, val / 250.0));
    if (reverse_on) x = -x;
    double y = std::sqrt(std::max(0.0, 1.0 - x*x)) * (reverse_on ? -1.0 : 1.0);
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

// Convenience: add a round button (cylinder) with a label
void push_button(MArray& arr,
    const std::string& frame, const rclcpp::Time& now, int& id,
    double px, double py,
    float r, float g, float b,
    const std::string& label,
    const rclcpp::Duration& lt)
{
    // Cylinder body
    arr.markers.push_back(make_marker(frame, now, id++, Marker::CYLINDER,
        px, py, 0.05,  0.55, 0.55, 0.10,  r, g, b, 1.0f, lt));
    // Label
    arr.markers.push_back(make_text(frame, now, id++, label,
        px, py, 0.18,  0.14,  1.0f, 1.0f, 1.0f, lt));
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

        reverse_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "Reverse_State", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                last_reverse_on_ = msg->data;
            });

        input_mode_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "Input_Mode", 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
                input_mode_ = static_cast<InputMode>(msg->data);
            });

        drive_mode_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "Drive_Mode", 10,
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
                drive_mode_ = static_cast<DriveMode>(msg->data);
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

        // ---- Decode state ----
        bool is_driving      = (std::abs(last_drive_) > 0.02);
        double bar_fill      = std::min(1.0, std::abs(last_drive_));
        bool sticks_neutral  = (std::abs(last_rotate_) < 2.0);

        double yaw      = val_to_display_yaw(last_rotate_, last_reverse_on_);
        std::string dir = sticks_neutral ? "CENTRE" : yaw_to_label(yaw);

        // ---- Colours per InputMode ----
        // LB button colour
        float lb_r, lb_g, lb_b;
        switch (input_mode_) {
            case InputMode::NONE:
                lb_r=1.0f; lb_g=0.1f; lb_b=0.1f; break;       // red
            case InputMode::JOYSTICK:
            case InputMode::GAMEPAD:
                lb_r=1.0f; lb_g=0.9f; lb_b=0.0f; break;       // yellow
            case InputMode::TURN_RIGHT:
            case InputMode::TURN_LEFT:
                lb_r=0.1f; lb_g=0.4f; lb_b=1.0f; break;       // blue
            case InputMode::PIVOT_HOME:
                lb_r=0.3f; lb_g=0.3f; lb_b=0.3f; break;       // grey
            default:
                lb_r=0.3f; lb_g=0.3f; lb_b=0.3f; break;
        }

        // B button colour + label
        float b_r, b_g, b_b;
        std::string b_label = "B";
        if (input_mode_ == InputMode::TURN_RIGHT) {
            b_r=0.1f; b_g=0.4f; b_b=1.0f; b_label="Turn\nRight";   // blue
        } else if (drive_mode_ == DriveMode::PIVOT_RIGHT && is_driving) {
            b_r=0.1f; b_g=0.4f; b_b=1.0f; b_label="Drive\nRight";  // blue
        } else {
            b_r=0.3f; b_g=0.3f; b_b=0.3f;
        }

        // X button colour + label
        float x_r, x_g, x_b;
        std::string x_label = "X";
        if (input_mode_ == InputMode::TURN_LEFT) {
            x_r=0.1f; x_g=0.4f; x_b=1.0f; x_label="Turn\nLeft";   // blue
        } else if (drive_mode_ == DriveMode::PIVOT_LEFT && is_driving) {
            x_r=0.1f; x_g=0.4f; x_b=1.0f; x_label="Drive\nLeft";  // blue
        } else {
            x_r=0.3f; x_g=0.3f; x_b=0.3f;
        }

        // Menu (PivotHome) button colour
        float m_r = (input_mode_ == InputMode::PIVOT_HOME) ? 0.1f : 0.3f;
        float m_g = (input_mode_ == InputMode::PIVOT_HOME) ? 0.4f : 0.3f;
        float m_b = (input_mode_ == InputMode::PIVOT_HOME) ? 1.0f : 0.3f;

        // Drive bar / MODE colour per DriveMode
        float dr_r, dr_g, dr_b;
        std::string mode_label;
        switch (drive_mode_) {
            case DriveMode::FORWARD:
                dr_r=0.1f; dr_g=0.9f; dr_b=0.2f; mode_label="FWD"; break;   // green
            case DriveMode::REVERSE:
                dr_r=1.0f; dr_g=0.1f; dr_b=0.1f; mode_label="REV"; break;   // red
            case DriveMode::PIVOT_RIGHT:
                dr_r=0.1f; dr_g=0.4f; dr_b=1.0f; mode_label="PIVOT\nRIGHT"; break; // blue
            case DriveMode::PIVOT_LEFT:
                dr_r=0.1f; dr_g=0.4f; dr_b=1.0f; mode_label="PIVOT\nLEFT";  break; // blue
            default:
                dr_r=0.3f; dr_g=0.3f; dr_b=0.3f; mode_label="IDLE";
        }
        if (!is_driving) { dr_r=0.3f; dr_g=0.3f; dr_b=0.3f; mode_label="IDLE"; }

        // ================================================================
        // PANEL 1 — DIRECTION  (centred x = -3)
        // ================================================================
        {
            const double X = -3.0;

            array.markers.push_back(make_marker(F, now, id++, Marker::CUBE,
                X, 0, -0.05,  2.4, 2.4, 0.05,  0.15f,0.15f,0.20f, 0.9f, lt));

            array.markers.push_back(make_text(F, now, id++, "DIRECTION",
                X, 1.35, 0.1,  0.25,  0.9f,0.9f,0.9f, lt));

            // Ring labels
            struct RL { const char* t; double dx, dy; };
            for (auto& rl : std::vector<RL>{
                    {"UP",    0.0,  0.85}, {"DOWN",  0.0, -0.85},
                    {"LEFT", -0.85, 0.0},  {"RIGHT", 0.85, 0.0}})
            {
                array.markers.push_back(make_text(F, now, id++, rl.t,
                    X + rl.dx, rl.dy, 0.1,  0.16,  0.5f,0.5f,0.5f, lt));
            }

            if (sticks_neutral) {
                // Show a dot instead of arrow when centred
                array.markers.push_back(make_marker(F, now, id++, Marker::SPHERE,
                    X, 0, 0.05,  0.18, 0.18, 0.18,  0.8f,0.8f,0.8f, 1.0f, lt));
            } else {
                // Rotating arrow
                auto arrow = make_marker(F, now, id++, Marker::ARROW,
                    X, 0, 0.05,  0.75, 0.10, 0.10,  1.0f,0.85f,0.0f, 1.0f, lt);
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
                    X, -1.35, 0.1,  0.21,  1.0f,0.85f,0.0f, lt));
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

            array.markers.push_back(make_marker(F, now, id++, Marker::CUBE,
                X, 0, -0.05,  2.4, 2.4, 0.05,  0.15f,0.15f,0.20f, 0.9f, lt));

            array.markers.push_back(make_text(F, now, id++, "DRIVE",
                X, 1.35, 0.1,  0.25,  0.9f,0.9f,0.9f, lt));

            // Track
            array.markers.push_back(make_marker(F, now, id++, Marker::CUBE,
                X, 0, 0.0,  BW+0.07, BH+0.07, 0.04,  0.08f,0.08f,0.08f, 1.0f, lt));

            // Fill — colour from DriveMode
            if (bar_fill > 0.01) {
                double fw  = bar_fill * BW;
                double fcx = B_LEFT + fw / 2.0;
                array.markers.push_back(make_marker(F, now, id++, Marker::CUBE,
                    fcx, 0, 0.03,  fw, BH, 0.04,  dr_r, dr_g, dr_b, 1.0f, lt));
            }

            // Ticks
            for (double t : {0.25, 0.50, 0.75}) {
                double tx = B_LEFT + t * BW;
                array.markers.push_back(make_marker(F, now, id++, Marker::CUBE,
                    tx, 0, 0.05,  0.025, BH*0.65, 0.05,  0.55f,0.55f,0.55f, 1.0f, lt));
            }

            array.markers.push_back(make_text(F, now, id++, "0",
                B_LEFT-0.14, 0, 0.1,  0.18,  0.6f,0.6f,0.6f, lt));
            array.markers.push_back(make_text(F, now, id++, "1",
                B_LEFT+BW+0.14, 0, 0.1,  0.18,  0.6f,0.6f,0.6f, lt));

            {
                char buf[32];
                snprintf(buf, sizeof(buf), "%.2f", last_drive_);
                array.markers.push_back(make_text(F, now, id++, buf,
                    X, -1.35, 0.1,  0.22,  0.7f,0.7f,0.7f, lt));
            }
        }

        // ================================================================
        // PANEL 3 — MODE  (centred x = +3)
        // ================================================================
        {
            const double X = 3.0;

            array.markers.push_back(make_marker(F, now, id++, Marker::CUBE,
                X, 0, -0.05,  2.4, 2.4, 0.05,  0.15f,0.15f,0.20f, 0.9f, lt));

            array.markers.push_back(make_text(F, now, id++, "MODE",
                X, 1.35, 0.1,  0.25,  0.9f,0.9f,0.9f, lt));

            array.markers.push_back(make_marker(F, now, id++, Marker::CYLINDER,
                X, 0, 0.05,  1.3, 1.3, 0.1,  dr_r, dr_g, dr_b, 1.0f, lt));

            array.markers.push_back(make_text(F, now, id++, mode_label,
                X, 0, 0.15,  0.28,  1.0f,1.0f,1.0f, lt));

            {
                char buf[32];
                snprintf(buf, sizeof(buf), "drive: %.2f", last_drive_);
                array.markers.push_back(make_text(F, now, id++, buf,
                    X, -1.35, 0.1,  0.20,  0.7f,0.7f,0.7f, lt));
            }
        }

        // ================================================================
        // BUTTON ROW  (y = -2.5, spaced along x)
        //   x=-5.5  LB
        //   x=-3.5  B  (Turn Right / Drive Right)
        //   x=-1.5  X  (Turn Left  / Drive Left)
        //   x= 0.5  Menu (Pivot Home)
        // ================================================================
        {
            const double BY = -2.5;

            // LB button
            push_button(array, F, now, id,  -5.5, BY,  lb_r, lb_g, lb_b,  "LB",  lt);

            // B button
            push_button(array, F, now, id,  -3.5, BY,  b_r,  b_g,  b_b,   b_label, lt);

            // X button
            push_button(array, F, now, id,  -1.5, BY,  x_r,  x_g,  x_b,   x_label, lt);

            // Menu / PivotHome button
            push_button(array, F, now, id,   0.5, BY,  m_r,  m_g,  m_b,   "Menu", lt);

            // Row label
            array.markers.push_back(make_text(F, now, id++, "BUTTONS",
                -2.5, BY + 0.85, 0.1,  0.20,  0.7f,0.7f,0.7f, lt));
        }

        marker_pub_->publish(array);
    }

    // Subscriptions
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr drive_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rotate_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr  reverse_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr input_mode_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr drive_mode_sub_;
    rclcpp::Publisher<MArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // State
    double    last_drive_      = 0.0;
    double    last_rotate_     = 0.0;
    bool      last_reverse_on_ = false;
    InputMode input_mode_      = InputMode::NONE;
    DriveMode drive_mode_      = DriveMode::FORWARD;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RoverVizNode>());
    rclcpp::shutdown();
    return 0;
}