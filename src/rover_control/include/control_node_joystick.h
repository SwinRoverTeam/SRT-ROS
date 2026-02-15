// control_node_joystick.h
// Swinburne Rover Team, 2026

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <iostream>
#include <cmath>

struct
{
    bool baseBool = false;
    bool shoulderBool = false;
    bool elbowBool = false;
    bool wristBool = false;
    bool rollBool = false;
} Arm;

class JoystickNode : public rclcpp::Node
{
private:
    int SendValBtn;

    int baseBtn;
    int shoulderBtn;
    int elbowBtn;
    int wristBtn;
    int rollBtn;

    bool baseValSent;
    bool shouldValSent = false;
    bool elbowValSent = false;
    bool wristValSent = false;
    bool rollValSent = false;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr JoystickSub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ArmJointsPub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ArmGripperPub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ExcavForkPub;

public:
    JoystickNode();

    double ScalingAlgorithm(double old_value, int new_min = 0, int new_max = 8, int old_min = 0, int old_max = 1);
};