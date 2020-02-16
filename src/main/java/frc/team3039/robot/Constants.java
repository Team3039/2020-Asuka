package frc.team3039.robot;

/*
 * A list of constants used by the rest of the robot code. This include physics
 * constants as well as constants determined through calibrations.
 */

public class Constants {
    public static final double kLooperDt = 0.01;

    // TODO: Tune Constants for 2020 Season
    // Wheels
    // 2020 Robot Values
    public static final double kDriveWheelTrackWidthInches = 28.00; // 22.61;
    public static final double kDriveWheelDiameterInches = 3.922; // 3.875
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
    public static final double kTrackScrubFactor = 0.9; // 0.924; // Tune me!

    // Tuned dynamics
    public static final double kRobotLinearInertia = 62.0; // kg
    public static final double kRobotAngularInertia = 10.0; // kg m^2
    public static final double kRobotAngularDrag = 12.0; // N*m / (rad/sec)
    public static final double kDriveVIntercept = 0.928112644250295; // V
    public static final double kDriveKv = 0.10305; // 0.14242500692715937; // V per rad/s
    public static final double kDriveKa = 0.01; // 0.011505866811140018; // V per rad/s^2

    //Turret
    public static final double kP_TURRET = 0.04;

    //Shooter
    public static final double kP_SHOOTER = 0.085;
    public static final double kI_SHOOTER = 0.0;
    public static final double kD_SHOOTER = 0.0;
    public static final double kF_SHOOTER = 0.0512;
    public static final int kIZone_SHOOTER = 200;
    public static final double SHOOTER_OUTPUT_TO_ENCODER_RATIO = 0.77; //Previous 3.0 Because 3 revolutions of the encoder was one revolution of the wheels, 24.0/36.0
    public static final double TICKS_PER_ROTATION = 2048.0;
    public static final int kLongCANTimeOutMs = 100;
	public static final double kFlywheelTicksPerRevolution = 0;

    // Geometry
    // 2020 Robot Values
    public static final double kCenterToFrontBumperDistance = 15.832;
    public static final double kCenterToRearBumperDistance = 15.832;
    public static final double kCenterToSideBumperDistance = 15.832;

    /* CONTROL LOOP GAINS */

    // Gearing and mechanical constants.
    public static final double kDriveDownShiftVelocity = 9.5 * 12.0; // inches per second
    public static final double kDriveDownShiftAngularVelocity = Math.PI / 2.0; // rad/sec
    public static final double kDriveUpShiftVelocity = 11.0 * 12.0; // inches per second

    public static final double kPathKX = 4.0;// 4.0; // units/s per unit of error
    public static final double kPathLookaheadTime = 0.4; // seconds to look ahead along the path for steering
    public static final double kPathMinLookaheadDistance = 24.0; // inches

    public static final double kDriveVelocityKp = 0.7; // 0.9;
    public static final double kDriveVelocityKi = 0.0;
    public static final double kDriveVelocityKd = 3.0; // 10.0;
    public static final double kDriveVelocityKf = 0.0;
    public static final int kDriveVelocityIZone = 0;
    public static final double kDriveVoltageRampRate = 0.0;
    public static double kDriveVelocityRampRate = 0.05; // 0.05; // 0.02
    public static double kDriveNominalOutput = 0.1;// 0.5 / 12.0;
    public static double kDriveMaxSetpoint = 11.0 * 12.0; // 11 fps


    /* I/O */
    // (Note that if multiple talons are dedicated to a mechanism, any sensors
    // are attached to the master)

    public static final int kCANTimeoutMs = 10; // use for on the fly updates
    public static final int kLongCANTimeoutMs = 100; // use for constructors

}
