// --------------------------------------------------------------------------------------------------------------------------
//  ROS2 code for the Flightstick
// --------------------------------------------------------------------------------------------------------------------------
//  Author: Disha Anchan
//  Last Updated: 16/03/2026
//  Description: This code contains the development for ROS2 node "Joystick", which converts the values received from the Flightstick 
//               (via the "Joy" node) to the respective value for the Arm and Excavation Fork payload of the Rover
//  --------------------------------------------------------------------------------------------------------------------------

#include "control_node_joystick.h"

JoystickNode::JoystickNode() : Node("Joystick")
{

  // --------------------------------------------------------------------------------------------------------------------------
  //  Topics made in the Joystick Node
  //
  //    - Arm_Joints = a Float64 array containing 5 elements that controls the Arm joints on the rover
  //        - Order of array: [Base, Shoulder, Elbow, Wrist, Roll]
  //        - Note: Roll is not used (as of 2026 season)        
  //
  //    - Arm_Gripper = a Float64 array that contains 2 elements that controls the opening and closing of the gripper
  //        Note: Gripper has not been made for the 2026 season; may be implement in future seasons.
  //
  //    - Excavation_Fork = a Float64 value that controls the lifts or lowers the excavation fork
  //  --------------------------------------------------------------------------------------------------------------------------

  // Following publishers have been created to control the Arm and Excavation Fork payload
  ArmJointsPub =
      create_publisher<std_msgs::msg::Float64MultiArray>("Arm_Joints", 10);       // Publisher for controlling the Arm Joints

  ArmGripperPub =
      create_publisher<std_msgs::msg::Float64MultiArray>("Arm_Gripper", 10);      // Publisher for controlling the Arm Gripper (opening and closing the gripper)

  ExcavForkPub =
      create_publisher<std_msgs::msg::Float64>("Excavation_Fork", 10);            // Publisher for controlling the Excavation_Fork (lifting and lowering )


  // Callback function for when a value from the Joy node has been received
  auto motor_callback =
      [this](const sensor_msgs::msg::Joy::SharedPtr msg) -> void
  {
    // Message structure used for for the respective topics 
    auto fork_msg = std_msgs::msg::Float64();                                   
    auto gripper_msg = std_msgs::msg::Float64MultiArray();                        
    auto arm_msg = std_msgs::msg::Float64MultiArray();

    // Publishing values under the topic "Arm_Gripper"
    slider_val = msg->axes[2];                                            // The Slider on the Flight Stick; Used for opening and closing the arm
    gripper_val = ScalingAlgorithm(slider_val, 0, 1, -1, 1);       // Scaling the value from a range between 0-1 to (-1) - 1
    gripper_msg.data = {gripper_val, gripper_val};              
    //ArmGripperPub->publish(gripper_msg);

    // Buttons used for the Arm_Joints
    SendValBtn = msg->buttons[0];                                         // Rapid fire Trigger (Red trigger); Used for sending the buttons 
    baseBtn = msg->buttons[1];                                            // Number "2" button: Used for controlling the "Base" of the Arm payload
    shoulderBtn = msg->buttons[4];                                        // Number "5" button: Used for controlling the "Shoulder" of the Arm
    elbowBtn = msg->buttons[2];                                           // Number "3" button: Used for controlling the "Elbow" of the Arm
    wristBtn = msg->buttons[3];                                           // Number "4" button: Used for controlling the "Wrist" of the Gripper
    rollBtn = msg->buttons[5];                                            // Number "6" button: Used for controlling the "Roll" of the Gripper [Not in Use]


    // Initially used for disengaging the Arm (No values can be sent to any of the joints); May be deleted later if never used.

    // if (disableBtn == 1) {
    //   Arm.baseBool = false;
    //   Arm.shoulderBool = false;
    //   Arm.elbowBool = false;
    //   Arm.wristBool = false;
    //   Arm.rollBool = false;
    // }

    // Selects only the Base joint to be controlled
    if (baseBtn == 1)
    {
      Arm.baseBool = true;
      Arm.shoulderBool = false;
      Arm.elbowBool = false;
      Arm.wristBool = false;
      Arm.rollBool = false;
    }

    // Selects only the Shoulder joint to be controlled
    if (shoulderBtn == 1)
    {
      Arm.baseBool = false;
      Arm.shoulderBool = true;
      Arm.elbowBool = false;
      Arm.wristBool = false;
      Arm.rollBool = false;
    }

    // Selects only the Elbow joint to be controlled
    if (elbowBtn == 1)
    {
      Arm.baseBool = false;
      Arm.shoulderBool = false;
      Arm.elbowBool = true;
      Arm.wristBool = false;
      Arm.rollBool = false;
    }

    // Selects only the Wrist joint to be controlled
    if (wristBtn == 1)
    {
      Arm.baseBool = false;
      Arm.shoulderBool = false;
      Arm.elbowBool = false;
      Arm.wristBool = true;
      Arm.rollBool = false;
    }

    // Selects only the Roll joint to be controlled
    if (rollBtn == 1)
    {
      Arm.baseBool = false;
      Arm.shoulderBool = false;
      Arm.elbowBool = false;
      Arm.wristBool = false;
      Arm.rollBool = true;
    }

    // Resets the flags (allows another value to be publish again)
    if (SendValBtn == 0)
    {
      baseValSent = false;
      shouldValSent = false;
      elbowValSent = false;
      wristValSent = false;
      rollValSent = false;
    }

    // Publish values under the Base joint or the Excavation Fork element (Element 1)
    if (Arm.baseBool)
    {
      // Base of the arm
      double base_val = msg->axes[1];                             //  axes[1] = Y direction of the Flight stick (Forward-Backward movement);
      arm_msg.data = {base_val, 0.0, 0.0, 0.0, 0.0};              //  all other elements remain 0 to prevent other joints from moving 


      // Excavation fork
      double fork_val = msg->axes[1];                             // axes[1] = Y direction of the Flight stick (Forward-Backward movement);
      fork_msg.data = fork_val;
      
      
      if (SendValBtn == 1)                                        // Only when the red trigger has been pressed
      {
        // Only sends the value once 
        while (baseValSent != true)
        {

          ExcavForkPub->publish(fork_msg);                        
          ArmJointsPub->publish(arm_msg);                         
          baseValSent = true;                                     
        }
      }
    }
    // Publish values under the Shoulder joint element (Element 2)
    else if (Arm.shoulderBool)
    {

      double shoulder_val = msg->axes[1];                         //  axes[1] = Y direction of the Flight stick (Forward-Backward movement);
      arm_msg.data = {0.0, shoulder_val, 0.0, 0.0, 0.0};          //  all other elements remain 0 to prevent other joints from moving

      if (SendValBtn == 1)                                        // Only when the red trigger has been pressed
      {
        // Only sends the value once
        while (shouldValSent != true)
        {

          ArmJointsPub->publish(arm_msg);
          shouldValSent = true;
        }
      }
    }
    // Publish values under the Elbow joint element (Element 3)
    else if (Arm.elbowBool)
    {

      double elbow_val = msg->axes[1];                          //  axes[1] = Y direction of the Flight stick (Forward-Backward movement);
      arm_msg.data = {0.0, 0.0, elbow_val, 0.0, 0.0};           //  all other elements remain 0 to prevent other joints from moving

      if (SendValBtn == 1)                                      // Only when the red trigger has been pressed
      {
        // Only sends the value once
        while (elbowValSent != true)                            
        {
          ArmJointsPub->publish(arm_msg);
          elbowValSent = true;
        }
      }
    }

    // Publish values under the Wrist joint element (Element 4)
    else if (Arm.wristBool)
    {

      double wrist_val = msg->axes[1];                          //  axes[1] = Y direction of the Flight stick (Forward-Backward movement);
      arm_msg.data = {0.0, 0.0, 0.0, wrist_val, 0.0};           //  all other elements remain 0 to prevent other joints from moving

      if (SendValBtn == 1)                                      //  Only when the red trigger has been pressed
      {
        // Only sends the value once
        while (wristValSent != true)
        {
          ArmJointsPub->publish(arm_msg);
          wristValSent = true;
        }
      }
    }

    // Publish values under the Roll joint element (Element 5)
    else if (Arm.rollBool)
    {

      double roll_val = msg->axes[1];                           //  axes[1] = Y direction of the Flight stick (Forward-Backward movement);
      arm_msg.data = {0.0, 0.0, 0.0, 0.0, roll_val};            //  all other elements remain 0 to prevent other joints from moving

      if (SendValBtn == 1)                                      //  Only when the red trigger has been pressed
      {
        // Only sends the value once
        while (rollValSent != true)
        {
          ArmJointsPub->publish(arm_msg);
          rollValSent = true;
        }
      }

    } 

    // Initially used for disengaging the Arm (No values can be sent to any of the joints); May be deleted later if never used.
    // else {

    //   arm_msg.data = { 0.0, 0.0, 0.0, 0.0, 0.0 };
    //   ArmJointsPub->publish(arm_msg);

    // }

  };

  // Subscriber is created to connect to the Joy Node under the topic "joy"
  // Note: the topic "joy" has been remapped to "joy_joystick" to allow both the Flightstick and Xbox controller
  //       to be connect to the ROS2 network simultaneously.

  JoystickSub =
    create_subscription<sensor_msgs::msg::Joy>("joy_joystick", 10, motor_callback);
}

// Scaling Algorithm: Scales the range from the Joy Node to its desired range
// Note: Only applicable for any controller that has an x and y axis or is under the property "axes" in the Joy Node 
//       (e.g. the joystick of the xbox controller, the gamepad of the Xbox, the flightStick)

// Parameters for the ScalingAlgorithm:
//    - old_value = value received from the Joy Node (must be under property "axes")   
//    - new_min   = minimum value for the new range   
//    - new_max   = maximum value for the new range  
//    - old_min   = minimum value reached by the controller axis (typically the value is -1) 
//    - old_max   = maximum value reached by the controller axis (typically the value is 1 )

double JoystickNode::ScalingAlgorithm(double old_value, int new_min, int new_max, int old_min, int old_max)
{

  double new_range = new_max - new_min;
  double new_value = (old_value - old_min) * new_range / (old_max - old_min) + new_min;
  return new_value;
  
}


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoystickNode>());
  rclcpp::shutdown();
  return 0;
}
