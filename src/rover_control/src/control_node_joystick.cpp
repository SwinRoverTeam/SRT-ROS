// -----------------------------------------------------------------
// Joystick Node
// -----------------------------------------------------------------

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <iostream>
#include <cmath>
using namespace std;

struct {
  bool baseBool = false;
  bool shoulderBool = false;
  bool elbowBool = false;
  bool wristBool = false;
  bool rollBool = false;
} Arm;

double ScalingAlgorithm(double old_value, int new_min = 0, int new_max = 8, int old_min = 0, int old_max = 1)
{

    if (old_max == old_min)
    {
        cout << "Original Minimum matches Original Maximum" << endl;
        return 0.0;
    }

    double new_range = new_max - new_min;
    double new_value = (old_value - old_min) * new_range / (old_max - old_min) + new_min;
    return new_value;
}


class JoystickNode : public rclcpp::Node
{
public:
  JoystickNode() : Node("Joystick")
  {

      RCLCPP_INFO(this->get_logger(), "Joystick Node has been activated");
      
      ArmJointsPub = 
        this->create_publisher<std_msgs::msg::Float64MultiArray>("Arm_Joints", 10);

      ArmGripperPub = 
        this->create_publisher<std_msgs::msg::Float64MultiArray>("Arm_Gripper", 10);
      
      auto motor_callback =
        [this](const sensor_msgs::msg::Joy::SharedPtr msg) -> void {

          auto gripper_msg = std_msgs::msg::Float64MultiArray();
          double gripper_val = ScalingAlgorithm(msg->axes[2],0,1,-1, 1);
          gripper_msg.data = {gripper_val, gripper_val};
          ArmGripperPub->publish(gripper_msg);
          
          auto arm_msg = std_msgs::msg::Float64MultiArray();

          SendValBtn = msg->buttons[0];
          baseBtn = msg->buttons[1];
          shoulderBtn = msg->buttons[4];
          elbowBtn = msg->buttons[2];
          wristBtn = msg->buttons[3];
          rollBtn = msg->buttons[5];

          // if (disableBtn == 1) {
          //   Arm.baseBool = false;
          //   Arm.shoulderBool = false;
          //   Arm.elbowBool = false;
          //   Arm.wristBool = false;
          //   Arm.rollBool = false;
          // } 

          if (baseBtn == 1) {
            Arm.baseBool = true;
            Arm.shoulderBool = false;
            Arm.elbowBool = false;
            Arm.wristBool = false;
            Arm.rollBool = false;
          } 

          if (shoulderBtn == 1) {
            Arm.baseBool = false;
            Arm.shoulderBool = true;
            Arm.elbowBool = false;
            Arm.wristBool = false;
            Arm.rollBool = false;
          } 
          
          if (elbowBtn == 1) {
            Arm.baseBool = false;
            Arm.shoulderBool = false;
            Arm.elbowBool = true;
            Arm.wristBool = false;
            Arm.rollBool = false;
          } 

          if (wristBtn == 1) {
            Arm.baseBool = false;
            Arm.shoulderBool = false;
            Arm.elbowBool = false;
            Arm.wristBool = true;
            Arm.rollBool = false;
          }

          if (rollBtn == 1) {
            Arm.baseBool = false;
            Arm.shoulderBool = false;
            Arm.elbowBool = false;
            Arm.wristBool = false;
            Arm.rollBool = true;
          }

          if (SendValBtn == 0) {

            baseValSent = false;
            shouldValSent = false;
            elbowValSent = false;
            wristValSent = false;
            rollValSent = false;

          }

          if (Arm.baseBool) {

            double base_val = msg->axes[1];
            arm_msg.data = { base_val, 0.0, 0.0, 0.0, 0.0 };

            if (SendValBtn == 1) {
              while (baseValSent != true) {

                ArmJointsPub->publish(arm_msg);
                baseValSent = true;

              }
            }

          }  else if (Arm.shoulderBool) {

            double shoulder_val = msg->axes[1];
            arm_msg.data = { 0.0, shoulder_val, 0.0, 0.0, 0.0 };

            if (SendValBtn == 1) {
              while (shouldValSent != true) {

                ArmJointsPub->publish(arm_msg);
                shouldValSent = true;

              }
            }

          } else if (Arm.elbowBool) {

            double elbow_val = msg->axes[1];
            arm_msg.data = { 0.0, 0.0, elbow_val, 0.0, 0.0 };

            if (SendValBtn == 1) {
              while (elbowValSent != true) {

                ArmJointsPub->publish(arm_msg);
                elbowValSent = true;

              }
            }            

          } else if (Arm.wristBool) {

            double wrist_val = msg->axes[1];
            arm_msg.data = { 0.0, 0.0, 0.0, wrist_val, 0.0 };

            if (SendValBtn == 1) {
              while (wristValSent != true) {

                ArmJointsPub->publish(arm_msg);
                wristValSent = true;

              }
            }

          } else if (Arm.rollBool) {

            double roll_val = msg->axes[1];
            arm_msg.data = { 0.0, 0.0, 0.0, 0.0, roll_val};

            if (SendValBtn == 1) {
              while (rollValSent != true) {

                ArmJointsPub->publish(arm_msg);
                rollValSent = true;

              }
            }

          } // else {

          //   arm_msg.data = { 0.0, 0.0, 0.0, 0.0, 0.0 };
          //   ArmJointsPub->publish(arm_msg);

          // }

        };

      JoystickSub =
        this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, motor_callback);
  }

private:
  // int disableBtn;

  int SendValBtn;

  int baseBtn;
  int shoulderBtn;
  int elbowBtn;
  int wristBtn;
  int rollBtn;

  bool baseValSent = false;
  bool shouldValSent = false;
  bool elbowValSent = false;
  bool wristValSent = false;
  bool rollValSent = false;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr JoystickSub;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ArmJointsPub;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ArmGripperPub;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoystickNode>());
  rclcpp::shutdown();
  return 0;

}
