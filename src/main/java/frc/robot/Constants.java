/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    //Drivetrain 
    public static final double DRIVER_Y = .85;
    public static final double DRIVER_ROT = .35;

    public static final double DRIVETRAIN_ENCODER_PPR = 2048.0; 
    public static final double DRIVE_GEAR_RATIO = 9; 
    public static final double WHEEL_DIAMETER = 5.875; 
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI; // pi * diameter
    public static final double DRIVE_PPR_TO_INCHES = WHEEL_CIRCUMFERENCE / (DRIVETRAIN_ENCODER_PPR * DRIVE_GEAR_RATIO); //How many inches is one encoder tick
    public static final double DRIVE_TRACKWIDTH = 26.5;

    //Turret
    public static final double kP_TURRET = 0.2;
    public static final double TURRET_ENCODER_PPR = 1; //TODO: Get this Number
    public static final double TURRET_GEAR_RATIO = 1; //TODO: Get this Number
    public static final double TURRET_PPR_TO_DEGREES = 360 / (TURRET_GEAR_RATIO * TURRET_ENCODER_PPR); //How many degrees is one tick

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
    
    //Color Wheel
    public static final double COLOR_WHEEL_PPR = 0;
    
 
}

