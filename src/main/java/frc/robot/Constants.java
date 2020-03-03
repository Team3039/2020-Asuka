/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */

 //TODO Add shooter speed constants 
public final class Constants {
    //Drive 
    public static final double DRIVER_Y = .85;
    public static final double DRIVER_ROT = .35;

    // 2020 Drive Constants
    public static final double kWheelDiameterInches = 5.835;
    public static final double kTrackWidthInches = 28.5;
    public static final double kDriveEncoderPPR = 2048;
    public static final double kEncoderRotationToWheelRotationRatio = 9.0/1.0;


    public static final double kTrackWidthMeters = Units.inchesToMeters(kTrackWidthInches);
    public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackWidthMeters);


    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
    // values for your robot.
    public static final double ksVolts = .535732026886109;
    public static final double kvVoltSecondsPerMeter = .036345975446567435;
    public static final double kaVoltSecondsSquaredPerMeter = .002053036858073132;

    // Example value only - as above, this must be tuned for your drive!
    public static final double kPDriveVel = 1.5; //8.5
    public static final double kDDriveVel = 0;


    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(5.0); //Find good value
    public static final double kMaxAccelerationMetersPerSecondSquared = Math.pow(Units.feetToMeters(5.0), 2);

    public static final double kMinSpeedMetersPerSecond = Units.feetToMeters(5.0); //Find good value
    public static final double kMinAccelerationMetersPerSecondSquared = Math.pow(Units.feetToMeters(5.0), 2);
    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2.0;
    public static final double kRamseteZeta = 0.7;

    //Turret
    public static final double kP_TURRET = -0.025;
    public static final double TURRET_RATIO = 15155.2;
    public static final double PPR = 3.7;
    public static final double TURRET_PPR_TO_DEGREES = 360 / TURRET_RATIO; //How many degrees is one tick

    //Shooter
    public static final double kP_SHOOTER = 0.085;
    public static final double kI_SHOOTER = 0.0;
    public static final double kD_SHOOTER = 0.0;
    public static final double kF_SHOOTER = 0.0512;
    public static final int kIZone_SHOOTER = 200;
    public static final double SHOOTER_OUTPUT_TO_ENCODER_RATIO = .44;
    public static final double TICKS_PER_ROTATION = 2048.0;
    public static final int kLongCANTimeOutMs = 100;
    public static final double kFlywheelTicksPerRevolution = 0;
    
    //Color Wheel
    public static final double COLOR_WHEEL_PPR = 0;

    public static final double TRIGGER_TOLERANCE = .1;
}

