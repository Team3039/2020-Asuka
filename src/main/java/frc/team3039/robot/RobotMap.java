package frc.team3039.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	// USB Port IDs
	public static final int DRIVER_JOYSTICK_1_USB_ID = 0;
	public static final int OPERATOR_JOYSTICK_1_USB_ID = 1;

	//Motors
	public static final int DRIVETRAIN_LEFT_MOTOR1_CAN_ID = 0;
	public static final int DRIVETRAIN_LEFT_MOTOR2_CAN_ID = 2;

	public static final int DRIVETRAIN_RIGHT_MOTOR1_CAN_ID = 1;
	public static final int DRIVETRAIN_RIGHT_MOTOR2_CAN_ID = 3;

	public static final int PIGEON_IMU_TALON_CAN_ID = 4;

	//Intake
	public static final int INTAKE_MOTOR_CAN_ID = 5;

	//Revolver
	public static final int HOPPER_REVOLVER_MOTOR_CAN_ID = 6;
	public static final int HOPPER_FEEDER_OMNI_CAN_ID = 7;
	public static final int HOPPER_KICKER_OMNI_CAN_ID = 8;
	public static final int HOPPER_FEEDER_BELTS_CAN_ID = 9;

	//Climb
	public static final int CLIMB_MOTOR_A_CAN_ID = 10;
	public static final int CLIMB_MOTOR_B_CAN_ID = 11;

	//Shooter
	public static final int SHOOTER_MOTOR_A_CAN_ID = 12;
	public static final int SHOOTER_MOTOR_B_CAN_ID = 13;
	public static final int TURRET_MOTOR_CAN_ID = 14;

	//Color Wheel
	public static final int CONTROL_PANEL_MOTOR_CAN_ID = 5;

	//Pneumatics
	public static final int INTAKE_TILT_PCM_ID = 1;
	public static final int CLIMB_DEPLOY_PCM_ID = 2;
	public static final int BUDDY_RELEASE_PCM_ID = 3;
	public static final int CONTROL_PANEL_DEPLOY_PCM_ID = 4;

}
