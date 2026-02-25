// ============================================================
// viz_node_controller.cpp
// Subscribes to /Pivot_Rotate and /Pivot_Drive and publishes
// /motor_markers (MarkerArray) for RViz verification display.
//
// Visuals:
//   1. Rotation arrow  — fixed circle, arrow sweeps ±90° mapped
//                        from ±250. Holds last value until next
//                        Pivot_Rotate message arrives.
//   2. LB indicator    — "STEERING ACTIVE" (green) if Pivot_Rotate
//                        received within last 0.5s, else "LOCKED" (grey).
//   3. Drive bar       — vertical bar, grows up (green) for forward,
//                        down (red) for reverse, proportional to
//                        Pivot_Drive magnitude (0.0 to 1.0).
//
// Pivot_Rotate range : -250.0 to +250.0  (arbitrary units)
//   mapped to compass : N=0 (top), E=+250 (right), W=-250 (left)
//   arrow sweeps W ↔ N ↔ E only (S is unreachable — wire limit)
// Pivot_Drive range  :  -1.0 to  +1.0
//   +ve = forward (green bar up)
//   -ve = reverse (red bar down)
// ============================================================

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <cmath>
#include <string>
#include <cstdio>

static constexpr double ROTATE_MAX   = 250.0;
static constexpr double VISUAL_MAX   = M_PI / 2.0;
static constexpr double ARROW_LEN    = 0.8;
static constexpr double CIRCLE_R     = 1.0;
static constexpr double BAR_MAX_H    = 0.8;
static constexpr double LB_TIMEOUT   = 0.5;
static constexpr double CIRCLE_X     =  0.0;
static constexpr double CIRCLE_Y     =  0.0;
static constexpr double BAR_X        =  2.5;
static constexpr double BAR_Y        =  0.0;

class VizBridge : public rclcpp::Node
{
public:
  VizBridge() : Node("viz_bridge"),
    last_rotate_(0.0),
    last_drive_(0.0)
  {
    rotate_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "Pivot_Rotate", 10,
      std::bind(&VizBridge::rotate_callback, this, std::placeholders::_1));

    drive_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
      "Pivot_Drive", 10,
      std::bind(&VizBridge::drive_callback, this, std::placeholders::_1));

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "motor_markers", 10);

    // Publish at 20 Hz so held-state visuals stay alive in RViz
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&VizBridge::publish_markers, this));

    last_rotate_time_ = this->now();

    RCLCPP_INFO(this->get_logger(),
      "viz_bridge started. Publishing /motor_markers.");
  }

private:
  void rotate_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (!msg->data.empty()) {
      last_rotate_ = msg->data[0];
      last_rotate_time_ = this->now();
    }
  }

  void drive_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (!msg->data.empty()) {
      last_drive_ = msg->data[0];
    }
  }

  void publish_markers()
  {
    visualization_msgs::msg::MarkerArray array;
    int id = 0;
    auto now = this->now();

    // ── 1. Reference circle ───────────────────────────────────────────────
    {
      visualization_msgs::msg::Marker circle;
      circle.header.frame_id = "base_link";
      circle.header.stamp    = now;
      circle.ns              = "ref_circle";
      circle.id              = id++;
      circle.type            = visualization_msgs::msg::Marker::LINE_STRIP;
      circle.action          = visualization_msgs::msg::Marker::ADD;
      circle.scale.x         = 0.02;
      circle.color           = makeColor(0.5f, 0.5f, 0.5f, 0.6f);
      circle.pose.orientation.w = 1.0;
      int steps = 64;
      for (int i = 0; i <= steps; i++) {
        double a = 2.0 * M_PI * i / steps;
        geometry_msgs::msg::Point p;
        p.x = CIRCLE_X + CIRCLE_R * std::cos(a);
        p.y = CIRCLE_Y + CIRCLE_R * std::sin(a);
        p.z = 0.0;
        circle.points.push_back(p);
      }
      array.markers.push_back(circle);
    }

    // ── 2. Rotation arrow ─────────────────────────────────────────────────
    // Compass convention: N=0 (top/+Y), E=+250 (right/+X), W=-250 (left/-X)
    // Mapping: arrow_angle (RViz radians from +X axis, CCW positive)
    //   N (rotate=0)    → π/2  (points to +Y)
    //   E (rotate=+250) → 0    (points to +X)
    //   W (rotate=-250) → π    (points to -X)
    //   Formula: arrow_angle = π/2 - (rotate/250) * (π/2)
    {
      double arrow_angle = M_PI / 2.0 - (last_rotate_ / ROTATE_MAX) * VISUAL_MAX;

      geometry_msgs::msg::Point start, end;
      start.x = CIRCLE_X; start.y = CIRCLE_Y; start.z = 0.0;
      end.x   = CIRCLE_X + ARROW_LEN * std::cos(arrow_angle);
      end.y   = CIRCLE_Y + ARROW_LEN * std::sin(arrow_angle);
      end.z   = 0.0;

      visualization_msgs::msg::Marker arrow;
      arrow.header.frame_id = "base_link";
      arrow.header.stamp    = now;
      arrow.ns              = "rotate_arrow";
      arrow.id              = id++;
      arrow.type            = visualization_msgs::msg::Marker::ARROW;
      arrow.action          = visualization_msgs::msg::Marker::ADD;
      arrow.points.push_back(start);
      arrow.points.push_back(end);
      arrow.scale.x = 0.06;
      arrow.scale.y = 0.12;
      arrow.scale.z = 0.12;
      arrow.color   = makeColor(1.0f, 0.85f, 0.0f, 1.0f);  // gold
      arrow.pose.orientation.w = 1.0;
      array.markers.push_back(arrow);

      // Compass labels N / E / S / W around the circle
      // S is greyed out — unreachable due to physical wire limit
      struct { const char* label; double lx; double ly; bool reachable; } compass[] = {
        { "N",  0.0,             CIRCLE_R + 0.15,  true  },
        { "E",  CIRCLE_R + 0.15, 0.0,              true  },
        { "S",  0.0,            -CIRCLE_R - 0.15,  false },
        { "W", -CIRCLE_R - 0.15, 0.0,              true  },
      };
      for (auto & c : compass) {
        visualization_msgs::msg::Marker clbl;
        clbl.header.frame_id = "base_link";
        clbl.header.stamp    = now;
        clbl.ns              = "compass_labels";
        clbl.id              = id++;
        clbl.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        clbl.action          = visualization_msgs::msg::Marker::ADD;
        clbl.pose.position.x = CIRCLE_X + c.lx;
        clbl.pose.position.y = CIRCLE_Y + c.ly;
        clbl.pose.position.z = 0.0;
        clbl.pose.orientation.w = 1.0;
        clbl.scale.z         = 0.18;
        clbl.color           = c.reachable
          ? makeColor(1.0f, 1.0f, 1.0f, 1.0f)
          : makeColor(0.4f, 0.4f, 0.4f, 0.6f);
        clbl.text            = c.label;
        array.markers.push_back(clbl);
      }

      // Rotate value label below the circle
      visualization_msgs::msg::Marker angle_label;
      angle_label.header.frame_id = "base_link";
      angle_label.header.stamp    = now;
      angle_label.ns              = "rotate_label";
      angle_label.id              = id++;
      angle_label.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      angle_label.action          = visualization_msgs::msg::Marker::ADD;
      angle_label.pose.position.x = CIRCLE_X;
      angle_label.pose.position.y = CIRCLE_Y - CIRCLE_R - 0.45;
      angle_label.pose.position.z = 0.0;
      angle_label.pose.orientation.w = 1.0;
      angle_label.scale.z         = 0.18;
      angle_label.color           = makeColor(1.0f, 1.0f, 1.0f, 1.0f);
      char buf[32];
      snprintf(buf, sizeof(buf), "ROTATE: %.1f", last_rotate_);
      angle_label.text            = buf;
      array.markers.push_back(angle_label);
    }

    // ── 3. LB indicator ───────────────────────────────────────────────────
    {
      double elapsed = (now - last_rotate_time_).seconds();
      bool   active  = (elapsed < LB_TIMEOUT);

      visualization_msgs::msg::Marker lb;
      lb.header.frame_id = "base_link";
      lb.header.stamp    = now;
      lb.ns              = "lb_indicator";
      lb.id              = id++;
      lb.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      lb.action          = visualization_msgs::msg::Marker::ADD;
      lb.pose.position.x = CIRCLE_X;
      lb.pose.position.y = CIRCLE_Y + CIRCLE_R + 0.25;
      lb.pose.position.z = 0.0;
      lb.pose.orientation.w = 1.0;
      lb.scale.z         = 0.2;
      lb.color           = active
        ? makeColor(0.0f, 1.0f, 0.3f, 1.0f)
        : makeColor(0.6f, 0.6f, 0.6f, 1.0f);
      lb.text            = active ? "STEERING ACTIVE" : "LOCKED";
      array.markers.push_back(lb);
    }

    // ── 4. Drive bar ──────────────────────────────────────────────────────
    {
      double drive   = last_drive_;
      double bar_h   = std::abs(drive) * BAR_MAX_H;
      bool   forward = (drive >= 0.0);

      visualization_msgs::msg::Marker bar;
      bar.header.frame_id = "base_link";
      bar.header.stamp    = now;
      bar.ns              = "drive_bar";
      bar.id              = id++;
      bar.type            = visualization_msgs::msg::Marker::CUBE;
      bar.action          = visualization_msgs::msg::Marker::ADD;
      bar.pose.position.x = BAR_X;
      bar.pose.position.y = BAR_Y + (forward ? 1.0 : -1.0) * (bar_h / 2.0);
      bar.pose.position.z = 0.0;
      bar.pose.orientation.w = 1.0;
      bar.scale.x         = 0.3;
      bar.scale.y         = bar_h > 0.01 ? bar_h : 0.01;
      bar.scale.z         = 0.1;
      bar.color           = forward
        ? makeColor(0.0f, 0.9f, 0.2f, 0.9f)
        : makeColor(0.9f, 0.1f, 0.1f, 0.9f);
      array.markers.push_back(bar);

      // Baseline at zero
      visualization_msgs::msg::Marker baseline;
      baseline.header.frame_id = "base_link";
      baseline.header.stamp    = now;
      baseline.ns              = "drive_baseline";
      baseline.id              = id++;
      baseline.type            = visualization_msgs::msg::Marker::CUBE;
      baseline.action          = visualization_msgs::msg::Marker::ADD;
      baseline.pose.position.x = BAR_X;
      baseline.pose.position.y = BAR_Y;
      baseline.pose.position.z = 0.0;
      baseline.pose.orientation.w = 1.0;
      baseline.scale.x         = 0.35;
      baseline.scale.y         = 0.03;
      baseline.scale.z         = 0.1;
      baseline.color           = makeColor(0.8f, 0.8f, 0.8f, 0.8f);
      array.markers.push_back(baseline);

      // Drive value + direction label (below bar)
      visualization_msgs::msg::Marker drive_label;
      drive_label.header.frame_id = "base_link";
      drive_label.header.stamp    = now;
      drive_label.ns              = "drive_label";
      drive_label.id              = id++;
      drive_label.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      drive_label.action          = visualization_msgs::msg::Marker::ADD;
      drive_label.pose.position.x = BAR_X;
      drive_label.pose.position.y = BAR_Y - BAR_MAX_H - 0.25;
      drive_label.pose.position.z = 0.0;
      drive_label.pose.orientation.w = 1.0;
      drive_label.scale.z         = 0.18;
      drive_label.color           = makeColor(1.0f, 1.0f, 1.0f, 1.0f);
      char dbuf[64];
      snprintf(dbuf, sizeof(dbuf), "DRIVE: %.2f  %s",
               std::abs(drive), forward ? "FWD" : "REV");
      drive_label.text            = dbuf;
      array.markers.push_back(drive_label);

      // DRIVE title label (above bar)
      visualization_msgs::msg::Marker drive_title;
      drive_title.header.frame_id = "base_link";
      drive_title.header.stamp    = now;
      drive_title.ns              = "drive_title";
      drive_title.id              = id++;
      drive_title.type            = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      drive_title.action          = visualization_msgs::msg::Marker::ADD;
      drive_title.pose.position.x = BAR_X;
      drive_title.pose.position.y = BAR_Y + BAR_MAX_H + 0.25;
      drive_title.pose.position.z = 0.0;
      drive_title.pose.orientation.w = 1.0;
      drive_title.scale.z         = 0.2;
      drive_title.color           = makeColor(1.0f, 1.0f, 1.0f, 0.8f);
      drive_title.text            = "DRIVE";
      array.markers.push_back(drive_title);
    }

    marker_pub_->publish(array);
  }

  std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a) {
    std_msgs::msg::ColorRGBA c; c.r=r; c.g=g; c.b=b; c.a=a; return c;
  }

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr rotate_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr drive_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double            last_rotate_;
  double            last_drive_;
  rclcpp::Time      last_rotate_time_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VizBridge>());
  rclcpp::shutdown();
  return 0;
}