// control_node_controller.h
// Swinburne Rover Team, 2026

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <iostream>
#include <cmath>
#include <vector>

double savedArr = 0.0;
bool reverseOn = false;
bool TurnRightMotor = false;
bool TurnLeftMotor = false;

class XboxCtrlNode : public rclcpp::Node
{
private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr triggerSub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr triggerPub;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystickSub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joystickPub;

    int SendValBtn;
    int TurnRightBtn;
    int TurnLeftBtn;

    double left_joystick_x;
    double left_joystick_y;
    double gamepad_x;
    double gamepad_y;

    bool TurnedLeft = false;
    bool TurnedRight = false;

public:
    XboxCtrlNode();

    double ScalingAlgorithm(double old_value, int new_min = 0, int new_max = 8, int old_min = 0, int old_max = 1);
    std::vector<double> JoystickAlgorithm(double x_axis, double y_axis);
    std::vector<double> MotorCompiler(double motor);
};