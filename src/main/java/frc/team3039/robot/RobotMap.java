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

	// Motors
	//Drive
	public static final int DRIVETRAIN_LEFT_MOTOR1_CAN_ID = 0;
	public static final int DRIVETRAIN_LEFT_MOTOR2_CAN_ID = 2;

	public static final int DRIVETRAIN_RIGHT_MOTOR1_CAN_ID = 1;
	public static final int DRIVETRAIN_RIGHT_MOTOR2_CAN_ID = 3;

	public static final int PIGEON_IMU_TALON = 4;

	//Revolver
	public static final int REVOLVER_SPIN_MOTOR_CAN_ID = 5;
	public static final int REVOLVER_FEEDER_OMNI_CAN_ID = 6;
	public static final int REVOLVER_KICKER_OMNI_CAN_ID = 7;
	public static final int REVOLVER_FEEDER_BELTS_CAN_ID = 8;



}
