// -----------------------------------------------------------------
// Xbox Controller Node
// -----------------------------------------------------------------

// Constexpr constants -> compile time constants

#include "control_node_controller.h"

XboxCtrlNode::XboxCtrlNode() : Node("XboxController") {

	RCLCPP_INFO(get_logger(), "XboxController Node has been activated");
	// Creating a publisher for the right trigger on the Xbox controller

	triggerPub = create_publisher<std_msgs::msg::Float64MultiArray>("Pivot_Drive", 10);

	auto trigger_callback = [this](const sensor_msgs::msg::Joy::SharedPtr msg) -> void {
		double right_trigger = msg->axes[4];

		auto trigger_msg = std_msgs::msg::Float64MultiArray();

		trigger_msg.data = MotorCompiler(right_trigger);

		triggerPub->publish(trigger_msg);
	};

	triggerSub = create_subscription<sensor_msgs::msg::Joy>("joy_xbox", 10, trigger_callback);

	// Creating a publisher for the left joystick on the Xbox controller

	joystickPub = create_publisher<std_msgs::msg::Float64MultiArray>("Pivot_Rotate", 10);

	auto joystick_callback = [this](const sensor_msgs::msg::Joy::SharedPtr msg) -> void {
		// to reduce the amount of messages received by the Joy Node (may be used for bandwidth reduction)

		// if (++count_ % 5 != 0) {
		//   return;
		// }

		left_joystick_x = msg->axes[0]; // X axis of the left joystick on the controller
		left_joystick_y = msg->axes[1]; // Y axis of the left joystick on the controller
		gamepad_x = msg->axes[6];       // X axis of the gamepad  on the controller
		gamepad_y = msg->axes[7];       // Y axis of the gamepad  on the controller

		SendValBtn = msg->buttons[6];   // LB button on the controller
		TurnRightBtn = msg->buttons[1]; // ( B ) button on the controller
		TurnLeftBtn = msg->buttons[3];  // ( X ) button on the controller

		auto joystick_msg = std_msgs::msg::Float64MultiArray();
		auto joystickArr = JoystickAlgorithm(left_joystick_y, left_joystick_x);
		auto gamepadArr = JoystickAlgorithm(gamepad_y, gamepad_x);

		if (TurnRightBtn == 0) {
			TurnedRight = false;
		}

		if (TurnLeftBtn == 0) {
			TurnedLeft = false;
		}

		if (SendValBtn == 1) {
			if ((savedArr > joystickArr[1] + 2 || savedArr < joystickArr[1] - 2) && gamepadArr[1] == 0 && (TurnRightBtn != 1 && TurnLeftBtn != 1)) {

				TurnLeftMotor = false;
				TurnRightMotor = false;
				auto joystick_msg = std_msgs::msg::Float64MultiArray();
				joystick_msg.data = joystickArr;
				joystickPub->publish(joystick_msg);
				savedArr = joystickArr[1];
			} else if ((savedArr > gamepadArr[1] + 2 || savedArr < gamepadArr[1] - 2) && joystickArr[1] == 0 && (TurnRightBtn != 1 && TurnLeftBtn != 1)) {

				TurnLeftMotor = false;
				TurnRightMotor = false;
				auto joystick_msg = std_msgs::msg::Float64MultiArray();
				joystick_msg.data = gamepadArr;
				joystickPub->publish(joystick_msg);
				savedArr = gamepadArr[1];
			} else if (TurnRightBtn == 1 && joystickArr[1] == 0.0 && gamepadArr[1] == 0.0) {

				TurnRightMotor = true;
				while (TurnedRight != true) {

					auto joystick_msg = std_msgs::msg::Float64MultiArray();
					joystick_msg.data = {-151.0, 151.0, 151.0, -151.0};
					joystickPub->publish(joystick_msg);
					savedArr = 600.0;
					TurnedRight = true;
				}
			} else if (TurnLeftBtn == 1 && joystickArr[1] == 0.0 && gamepadArr[1] == 0.0) {

				TurnLeftMotor = true;
				while (TurnedLeft != true) {

					auto joystick_msg = std_msgs::msg::Float64MultiArray();
					joystick_msg.data = {-151.0, 151.0, 151.0, -151.0};
					joystickPub->publish(joystick_msg);
					savedArr = -600.0;
					TurnedLeft = true;
				}
			}
		}
	};

	joystickSub = create_subscription<sensor_msgs::msg::Joy>("joy_xbox", 10, joystick_callback);
}

double XboxCtrlNode::ScalingAlgorithm(double old_value, int new_min, int new_max, int old_min, int old_max) {
	if (old_max == old_min) {
		std::cout << "Original Minimum matches Original Maximum" << std::endl;
		return 0.0;
	}

	double new_range = new_max - new_min;
	double new_value = (old_value - old_min) * new_range / (old_max - old_min) + new_min;

	return new_value;
}

std::vector<double> XboxCtrlNode::JoystickAlgorithm(double x_axis, double y_axis) {
	double rad = -atan2(y_axis, x_axis); // Archtan Math
	// double deg = (rad * (180 / M_PI));                          // Fix needed with the value of Pi (90 degrees in place to correct for earlier detected issue
	// of motor range)

	if (x_axis == -0.0 && y_axis == -0.0) {
		return {0.0, 0.0, 0.0, 0.0};
	} else {

		double val = 500 * (rad / M_PI);

		if (val < -250.0) {
			val += 500;
			reverseOn = true;
		} else if (val > 250.0) {
			val -= 500;
			reverseOn = true;
		} else {
			reverseOn = false;
		}

		// return {deg, deg, deg, deg}
		return std::vector<double>{val, val, val, val};
	}
}

std::vector<double> XboxCtrlNode::MotorCompiler(double motor) {
	double mtr_forward = ScalingAlgorithm(motor, 0, 1, 1, -1);
	double mtr_reverse = ScalingAlgorithm(motor, 0, -1, 1, -1);

	if (reverseOn && !TurnRightMotor && !TurnLeftMotor) {
		return std::vector<double>{mtr_reverse, mtr_reverse, mtr_reverse, mtr_reverse};
	}

	else if (!reverseOn && TurnRightMotor && !TurnLeftMotor) {
		return std::vector<double>{mtr_reverse, mtr_forward, mtr_reverse, mtr_forward};
	}

	else if (!reverseOn && !TurnRightMotor && TurnLeftMotor) {
		return std::vector<double>{mtr_forward, mtr_reverse, mtr_forward, mtr_reverse};
	}

	else {
		return std::vector<double>{mtr_forward, mtr_forward, mtr_forward, mtr_forward};
	}
}

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<XboxCtrlNode>());
	rclcpp::shutdown();
	return 0;
}
