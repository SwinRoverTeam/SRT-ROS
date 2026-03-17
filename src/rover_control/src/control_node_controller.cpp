// -----------------------------------------------------------------
// Xbox Controller Node
// -----------------------------------------------------------------

// Constexpr constants -> compile time constants

#include "control_node_controller.h"

// Input mode: encodes which control is active this callback tick.
// Evaluated once per joystick_callback so every switch reads cleanly.
enum class InputMode {
	NONE,           // LB not held, or nothing changed
	JOYSTICK,       // Left joystick moved (LB held)
	GAMEPAD,        // D-pad moved (LB held)
	TURN_RIGHT,     // B button (LB held, sticks neutral)
	TURN_LEFT,      // X button (LB held, sticks neutral)
	PIVOT_HOME      // Menu button (LB held, sticks neutral, no B/X)
};

// Drive mode: encodes the current wheel-orientation state for MotorCompiler.
enum class DriveMode {
	FORWARD,        // Wheels facing front 180° — normal drive
	REVERSE,        // Wheels facing rear 180° — reversed drive
	PIVOT_RIGHT,    // B-button pivot
	PIVOT_LEFT      // X-button pivot
};

XboxCtrlNode::XboxCtrlNode() : Node("XboxController") {

	RCLCPP_INFO(get_logger(), "XboxController Node has been activated");
	
	// Trigger callback — right trigger drives motors (independent of LB)
	triggerPub = create_publisher<std_msgs::msg::Float64MultiArray>("Pivot_Drive", 10);

	auto trigger_callback = [this](const sensor_msgs::msg::Joy::SharedPtr msg) -> void {
		double right_trigger = msg->axes[4];

		auto trigger_msg = std_msgs::msg::Float64MultiArray();
		trigger_msg.data = MotorCompiler(right_trigger);
		triggerPub->publish(trigger_msg);
	};

	// Joystick / button callback — wheel steering, Pivot Home, and driving state for RVIZ
	triggerSub = create_subscription<sensor_msgs::msg::Joy>("joy_xbox", 10, trigger_callback);

	joystickPub  = create_publisher<std_msgs::msg::Float64MultiArray>("Pivot_Rotate", 10); // For wheel steering angles
	PivotHomePub = create_publisher<std_msgs::msg::Bool>("Pivot_Home", 10); // For homing signal to pivot node
	reverseStatePub = create_publisher<std_msgs::msg::Bool>("Reverse_State", 10); // For RVIZ to know which half of the joystick is active
	inputModePub    = create_publisher<std_msgs::msg::Int32>("Input_Mode", 10); // For RVIZ to know the steering angles 
	driveModePub    = create_publisher<std_msgs::msg::Int32>("Drive_Mode", 10); // For RVIZ to know the drive direction

	auto joystick_callback = [this](const sensor_msgs::msg::Joy::SharedPtr msg) -> void {
		// to reduce the amount of messages received by the Joy Node (may be used for bandwidth reduction)

		// if (++count_ % 5 != 0) {
		//   return;
		// }

		left_joystick_x = msg->axes[0];  // X axis of the left joystick on the controller
		left_joystick_y = msg->axes[1];  // Y axis of the left joystick on the controller
		gamepad_x       = msg->axes[6];  // X axis of the gamepad  on the controller
		gamepad_y       = msg->axes[7];  // Y axis of the gamepad  on the controller

		SendValBtn   = msg->buttons[6];  // LB button on the controller
		TurnRightBtn = msg->buttons[1];  // ( B ) button on the controller
		TurnLeftBtn  = msg->buttons[3];  // ( X ) button on the controller
		PivotHomeBtn = msg->buttons[11]; // Menu button — homing the Pivots of the rover

		// Reset one-shot latch flags when their buttons are released
		if (TurnRightBtn == 0) { TurnedRight = false; }
		if (TurnLeftBtn  == 0) { TurnedLeft  = false; }
		if (PivotHomeBtn == 0) { PivotHomed  = false; }

		// Pivot_Rotate only valid when LB is pressed
		if (SendValBtn != 1) { return; }

		// reserveOn state is preserved until next instruction (LB == 1), for MotorCompiler to correctly act on the latest wheel orientation 
		auto joystickArr = JoystickAlgorithm(left_joystick_y, left_joystick_x);
		auto gamepadArr  = JoystickAlgorithm(gamepad_y, gamepad_x);

		//bool reverseOnAfterJoystick = reverseOn; // snapshot before gamepad overwrites it
		// Restore: joystick snapshot wins until a publish path decides otherwise
		//reverseOn = reverseOnAfterJoystick;

		// Map the various states with correspond controller value
		bool joystickMoved = (savedArr > joystickArr[1] + 2 || savedArr < joystickArr[1] - 2);
		bool gamepadMoved  = (savedArr > gamepadArr[1]  + 2 || savedArr < gamepadArr[1]  - 2);
		bool sticksNeutral = (joystickArr[1] == 0.0 && gamepadArr[1] == 0.0);
		bool noPivotBtn    = (TurnRightBtn != 1 && TurnLeftBtn != 1);

		InputMode mode = InputMode::NONE;

		// joystick has priority over gamepad, the && coniditon prvents conflict between sticks and buttons
		if      (joystickMoved && gamepadArr[1] == 0 && noPivotBtn)  { mode = InputMode::JOYSTICK;   }
		else if (gamepadMoved && joystickArr[1] == 0 && noPivotBtn)  { mode = InputMode::GAMEPAD;    }
		else if (TurnRightBtn == 1 && sticksNeutral)                 { mode = InputMode::TURN_RIGHT; }
		else if (TurnLeftBtn  == 1 && sticksNeutral)                 { mode = InputMode::TURN_LEFT;  }
		else if (PivotHomeBtn == 1 && sticksNeutral && noPivotBtn)   { mode = InputMode::PIVOT_HOME; }

		switch (mode) {

			case InputMode::JOYSTICK: {
				JoystickAlgorithm(left_joystick_y, left_joystick_x);
				TurnLeftMotor  = false;
				TurnRightMotor = false;
				auto joystick_msg = std_msgs::msg::Float64MultiArray();
				joystick_msg.data = joystickArr;
				joystickPub->publish(joystick_msg);
				savedArr = joystickArr[1];
				break;
			}

			case InputMode::GAMEPAD: {
				JoystickAlgorithm(gamepad_y, gamepad_x);
				TurnLeftMotor  = false;
				TurnRightMotor = false;
				auto joystick_msg = std_msgs::msg::Float64MultiArray();
				joystick_msg.data = gamepadArr;
				joystickPub->publish(joystick_msg);
				savedArr = gamepadArr[1];
				break;
			}

			case InputMode::TURN_RIGHT: {
				TurnRightMotor = true;
				TurnLeftMotor  = false;
				reverseOn = false; // ensure reverseOn is false when pivoting
				if (!TurnedRight) {
					auto joystick_msg = std_msgs::msg::Float64MultiArray();
					joystick_msg.data = {-151.0, 151.0, 151.0, -151.0};
					joystickPub->publish(joystick_msg);
					savedArr    = 600.0;
					TurnedRight = true;
				}
				break;
			}

			case InputMode::TURN_LEFT: {
				TurnLeftMotor  = true;
				TurnRightMotor = false;
				reverseOn = false; // ensure reverseOn is false when pivoting
				if (!TurnedLeft) {
					auto joystick_msg = std_msgs::msg::Float64MultiArray();
					joystick_msg.data = {-151.0, 151.0, 151.0, -151.0};
					joystickPub->publish(joystick_msg);
					savedArr   = -600.0;
					TurnedLeft = true;
				}
				break;
			}

			case InputMode::PIVOT_HOME: {
				if (!PivotHomed) {
					auto pivotHome_msg = std_msgs::msg::Bool();
					pivotHome_msg.data = true;
					PivotHomePub->publish(pivotHome_msg);
					PivotHomed = true;
				}
				break;
			}

			case InputMode::NONE:
			default:
				// LB held but no input changed — do nothing
				break;
		}

		// Publish the state to the RVIZ node
		auto im_msg = std_msgs::msg::Int32();
		im_msg.data = static_cast<int>(mode);
		inputModePub->publish(im_msg);
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
	double rad = -atan2(y_axis, x_axis); // Arctangent math

	if (x_axis == -0.0 && y_axis == -0.0) {
		return {0.0, 0.0, 0.0, 0.0};
	}

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

	// Publish the state to the RVIZ node
	auto rev_msg = std_msgs::msg::Bool();
	rev_msg.data = reverseOn;
	reverseStatePub->publish(rev_msg);

	return std::vector<double>{val, val, val, val};
}

std::vector<double> XboxCtrlNode::MotorCompiler(double motor) {
	double mtr_forward = ScalingAlgorithm(motor, 0,  1, 1, -1);
	double mtr_reverse = ScalingAlgorithm(motor, 0, -1, 1, -1);

	// Default to FORWARD
	DriveMode driveMode = DriveMode::FORWARD;

	if      (TurnRightMotor && !TurnLeftMotor && !reverseOn) { driveMode = DriveMode::PIVOT_RIGHT; }
	else if (!TurnRightMotor && TurnLeftMotor && !reverseOn) { driveMode = DriveMode::PIVOT_LEFT;  }
	else if (reverseOn && !TurnLeftMotor && !TurnRightMotor) { driveMode = DriveMode::REVERSE;     }
	else if (!reverseOn && !TurnLeftMotor && !TurnRightMotor){ driveMode = DriveMode::FORWARD;     }

	// Publish the state to the RVIZ node
	auto dm_msg = std_msgs::msg::Int32();
	dm_msg.data = static_cast<int>(driveMode);
	driveModePub->publish(dm_msg);

	switch (driveMode) {

		case DriveMode::REVERSE:
			return std::vector<double>{mtr_reverse, mtr_reverse, mtr_reverse, mtr_reverse};
		break;

		case DriveMode::PIVOT_RIGHT:
			return std::vector<double>{mtr_reverse, mtr_forward, mtr_reverse, mtr_forward};
		break;

		case DriveMode::PIVOT_LEFT:
			return std::vector<double>{mtr_forward, mtr_reverse, mtr_forward, mtr_reverse};
		break;
		
		case DriveMode::FORWARD: {
			return std::vector<double>{mtr_forward, mtr_forward, mtr_forward, mtr_forward};
		break;

		default:
			// Fail-safe default case: does nothing if none of the above are satisfied
			return std::vector<double>{0.0, 0.0, 0.0, 0.0};
			break;
		}
		// Fail-safe fallback for any unexpected state.
		return std::vector<double>{0.0, 0.0, 0.0, 0.0};
	}
}

int main(int argc, char* argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<XboxCtrlNode>());
	rclcpp::shutdown();
	return 0;
}