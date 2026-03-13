// -----------------------------------------------------------------
// Xbox Controller Node
// -----------------------------------------------------------------

#include "control_node_controller.h"

// Xbox controller node: converts Xbox Joy messages into rover commands.
// - `Pivot_Drive` is driven by the right trigger axis (axis 4).
// - `Pivot_Rotate` is produced from the left joystick/gamepad inputs.
// - `SendValBtn` (LB / button 6) gates publishing for safety so the
//   motors only receive commands when LB is actively pressed.
XboxCtrlNode::XboxCtrlNode() : Node("XboxController") {

	RCLCPP_INFO(get_logger(), "XboxController Node has been activated");

	triggerPub = create_publisher<std_msgs::msg::Float64MultiArray>("Pivot_Drive", 10);

	// Trigger callback: converts right trigger axis to a drive command.
	auto trigger_callback = [this](const sensor_msgs::msg::Joy::SharedPtr msg) -> void {
		// Axis 4 is right trigger for this controller profile.
		double right_trigger = msg->axes[4];
		auto trigger_msg = std_msgs::msg::Float64MultiArray();
		trigger_msg.data = MotorCompiler(right_trigger);
		triggerPub->publish(trigger_msg);
	};

	triggerSub = create_subscription<sensor_msgs::msg::Joy>("joy_xbox", 10, trigger_callback);

	joystickPub = create_publisher<std_msgs::msg::Float64MultiArray>("Pivot_Rotate", 10);
	PivotHomePub = create_publisher<std_msgs::msg::Bool>("Pivot_Home", 10);

	// Joystick callback: handles pivot rotation, turning and homing.
	// - `SendValBtn` (LB) gates publishing; when released flags are reset.
	// - Turn/Homing actions are single-shot via flags (e.g. `TurnedRight`).
	auto joystick_callback = [this](const sensor_msgs::msg::Joy::SharedPtr msg) -> void {
		// Left stick and D-pad are both accepted as rotation inputs.
		left_joystick_x = msg->axes[0];
		left_joystick_y = msg->axes[1];
		gamepad_x = msg->axes[6];
		gamepad_y = msg->axes[7];

		// LB (button 6) is the publish-enable button for pivot commands.
		// Keep LB as a digital button; do not mix with trigger axes.
		SendValBtn = msg->buttons[6];
		TurnRightBtn = msg->buttons[1];
		TurnLeftBtn = msg->buttons[3];
		PivotHomeBtn = msg->buttons[11];

		auto joystickArr = JoystickAlgorithm(left_joystick_y, left_joystick_x);
		auto gamepadArr = JoystickAlgorithm(gamepad_y, gamepad_x);

		// Re-arm single-shot actions once their button is released.
		if (TurnRightBtn == 0) {
			TurnedRight = false;
		}
		if (TurnLeftBtn == 0) {
			TurnedLeft = false;
		}
		if (PivotHomeBtn == 0) {
			PivotHomed = false;
		}

		if (SendValBtn == 1) {
			// Publish only when the target value changed enough to matter.
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
				// Turn mode is latched and sent once, then maintained by state.
				TurnRightMotor = true;
				TurnLeftMotor = false;
				reverseOn = false;
				if (!TurnedRight) {
					auto joystick_msg = std_msgs::msg::Float64MultiArray();
					joystick_msg.data = {-151.0, 151.0, 151.0, -151.0};
					joystickPub->publish(joystick_msg);
					savedArr = 600.0;
					TurnedRight = true;
				}

			} else if (TurnLeftBtn == 1 && joystickArr[1] == 0.0 && gamepadArr[1] == 0.0) {
				// Mirror of right-turn behavior for left-turn command.
				TurnLeftMotor = true;
				TurnRightMotor = false;
				reverseOn = false;
				if (!TurnedLeft) {
					auto joystick_msg = std_msgs::msg::Float64MultiArray();
					joystick_msg.data = {-151.0, 151.0, 151.0, -151.0};
					joystickPub->publish(joystick_msg);
					savedArr = -600.0;
					TurnedLeft = true;
				}

			} else if (PivotHomeBtn == 1 && joystickArr[1] == 0.0 && gamepadArr[1] == 0.0 && (TurnRightBtn != 1 && TurnLeftBtn != 1)) {
				// Home is also single-shot while the button remains held.
				if (!PivotHomed) {
					auto pivotHome_msg = std_msgs::msg::Bool();
					pivotHome_msg.data = true;
					PivotHomePub->publish(pivotHome_msg);
					PivotHomed = true;
				}

			} else {
				// If no explicit rotate/turn/home command is active, clear mode flags.
				reverseOn = false;
				TurnRightMotor = false;
				TurnLeftMotor = false;
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
	// Linear map from [old_min, old_max] into [new_min, new_max].
	double new_range = new_max - new_min;
	double new_value = (old_value - old_min) * new_range / (old_max - old_min) + new_min;
	return new_value;
}

std::vector<double> XboxCtrlNode::JoystickAlgorithm(double x_axis, double y_axis) {
	// Convert stick/cartesian input into an angle-derived steering scalar.
	double rad = -atan2(y_axis, x_axis);

	if (x_axis == -0.0 && y_axis == -0.0) {
		return {0.0, 0.0, 0.0, 0.0};
	}

	double val = 500 * (rad / M_PI);

	// Wrap to keep command in the expected pivot range and set reverse state.
	if (val < -250.0) {
		val += 500;
		reverseOn = true;
	} else if (val > 250.0) {
		val -= 500;
		reverseOn = true;
	} else {
		reverseOn = false;
	}

	return {val, val, val, val};
}

std::vector<double> XboxCtrlNode::MotorCompiler(double motor) {
	// Generate forward/reverse throttle candidates from trigger position.
	double mtr_forward = ScalingAlgorithm(motor, 0, 1, 1, -1);
	double mtr_reverse = ScalingAlgorithm(motor, 0, -1, 1, -1);

	// Combine drive direction with turn mode to produce 4-wheel command outputs.
	if (reverseOn && !TurnRightMotor && !TurnLeftMotor) {
		return {mtr_reverse, mtr_reverse, mtr_reverse, mtr_reverse};
	} else if (!reverseOn && TurnRightMotor && !TurnLeftMotor) {
		return {mtr_reverse, mtr_forward, mtr_reverse, mtr_forward};
	} else if (!reverseOn && !TurnRightMotor && TurnLeftMotor) {
		return {mtr_forward, mtr_reverse, mtr_forward, mtr_reverse};
	} else {
		return {mtr_forward, mtr_forward, mtr_forward, mtr_forward};
	}
}

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<XboxCtrlNode>());
	rclcpp::shutdown();
	return 0;
}
