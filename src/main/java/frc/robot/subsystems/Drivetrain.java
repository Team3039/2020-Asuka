/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.DRIVER_ROT;
import static frc.robot.Constants.DRIVER_Y;
import static frc.robot.Constants.DRIVE_PPR_TO_INCHES;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.controllers.PS4Gamepad;

/**
 * The Drivetrain Class contains the wheelbase's motors and all sensor
 * information for the robot's position on the field.
 */
public class Drivetrain extends SubsystemBase {

  public static TalonFX leftFrontDrive = new TalonFX(RobotMap.leftFrontDrive);
  public static TalonFX leftRearDrive = new TalonFX(RobotMap.leftRearDrive);
  public static TalonFX rightFrontDrive = new TalonFX(RobotMap.rightFrontDrive);
  public static TalonFX rightRearDrive = new TalonFX(RobotMap.rightRearDrive);
  public static AHRS navX = new AHRS(SPI.Port.kMXP);
  
  public static PIDController driveCtrl = new PIDController(.012,0,0);
  public static PIDController rotCtrl = new PIDController(.0165, 0, 0);

  public double kPGyro = 0.0287;
  
  public Drivetrain() {
    setNeutralMode(NeutralMode.Brake);
    rightRearDrive.follow(rightFrontDrive);
    leftRearDrive.follow(leftFrontDrive);
    leftRearDrive.configSelectedFeedbackCoefficient(DRIVE_PPR_TO_INCHES);
    rightRearDrive.configSelectedFeedbackCoefficient(DRIVE_PPR_TO_INCHES);
    resetEncoders();
  }

  public enum DriveControl {
      TELEOP,
      AUTO
  }

  public void joystickControl(PS4Gamepad gp) {
    // Tele-Op Driving
    // Each Motor is Set to Brake Mode, the motor speeds are set in an Arcade Drive
    // fashion
    double y = gp.getLeftYAxis() * DRIVER_Y;
    double rot = -1 * gp.getRightXAxis() * DRIVER_ROT;

    // Calculated Outputs (Limits Output to 12V)
    double leftOutput = y + rot;
    double rightOutput = rot - y;

    // Assigns Each Motor's Power
    leftFrontDrive.set(ControlMode.PercentOutput, leftOutput);
    rightFrontDrive.set(ControlMode.PercentOutput, rightOutput);
  }

  public void stop() {
    leftFrontDrive.set(ControlMode.PercentOutput, 0);
    rightFrontDrive.set(ControlMode.PercentOutput, 0);
  }

  public void setNeutralMode(NeutralMode mode) {
    // Set Motor's Neutral/Idle Mode to Brake or Coast
    leftFrontDrive.setNeutralMode(mode);
    leftRearDrive.setNeutralMode(mode);
    rightFrontDrive.setNeutralMode(mode);
    rightRearDrive.setNeutralMode(mode);
  }

  public void resetEncoders() {
    leftRearDrive.setSelectedSensorPosition(0);
    rightRearDrive.setSelectedSensorPosition(0);
    leftRearDrive.setSensorPhase(false);
    rightRearDrive.setSensorPhase(false);
  }

  public double getLeftPosition() {
    return -leftRearDrive.getSelectedSensorPosition();
  }

  public double getRightPosition() {
    return rightRearDrive.getSelectedSensorPosition();
  }

  public double getLeftVelocity() {
    return -leftRearDrive.getSelectedSensorVelocity();
  }

  public double getRightVelocity() {
    return rightRearDrive.getSelectedSensorVelocity();
  }

  public double inchesToMeters(double inches) {
    return inches * 0.0254;
  }

  public double degToRad(double degrees) {
    return degrees * Math.PI / 180;
  }

  public void resetGyro() {
    navX.reset();
  }

  public double getAngle() {
    return navX.getAngle();
  }

  public double getAvgDistance() {
    return (getLeftPosition() + getRightPosition()) / 2.0;
  }

  public void drivePID(double distance) {
    double output = (driveCtrl.calculate(getAvgDistance(), distance));
    double clampedOutput = MathUtil.clamp(output, -.3, .3);

    if (distance >= 0) {
      driveStraight(clampedOutput);
    }
    else {
      driveStraight(clampedOutput);
    }
  }

  public void driveStraight(double power) {
    double gyroError = 0 - getAngle();
    double kP = kPGyro;
    leftFrontDrive.set(ControlMode.PercentOutput, -power - (kP * gyroError) );
    rightFrontDrive.set(ControlMode.PercentOutput, power - (kP * gyroError) );
  }

  public void reverseStraight(double power) {
    double gyroError = 0 - getAngle();
    double kP = 0;
    leftFrontDrive.set(ControlMode.PercentOutput, power + (kP * gyroError) );
    rightFrontDrive.set(ControlMode.PercentOutput, power + (kP * gyroError) );
  }

  public void rotatePID(double angle) {
    double output = (rotCtrl.calculate(getAngle(), angle));
    double clampedOutput = MathUtil.clamp(output, -.3, .3);
    System.out.println(getRotError());
    if (getAngle() < angle) {
    rotate(-clampedOutput);
    }
  }

  public double getDrvError() {
    return driveCtrl.getPositionError();
  }

  public double getRotError() {
    return rotCtrl.getPositionError();
  }

  public void rotate(double power) {
    leftFrontDrive.set(ControlMode.PercentOutput, power);
    rightFrontDrive.set(ControlMode.PercentOutput, power);
  }

  public void configDrivePID() {
    leftFrontDrive.config_kP(0, .02);
    rightFrontDrive.config_kP(0, .02);
    leftFrontDrive.config_kI(0, 0);
    rightFrontDrive.config_kI(0, 0);
    leftFrontDrive.config_kD(0, 0);
    rightFrontDrive.config_kD(0, 0);
  }

  public void configTurnPID() {
    leftFrontDrive.config_kP(0, 0);
    rightFrontDrive.config_kP(0, 0);
    leftFrontDrive.config_kI(0, 0);
    rightFrontDrive.config_kI(0, 0);
    leftFrontDrive.config_kD(0, 0);
    rightFrontDrive.config_kD(0, 0);
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Position", getLeftPosition());
    SmartDashboard.putNumber("Right Position", getRightPosition());
    SmartDashboard.putNumber("Left Velocity", getLeftVelocity());
    SmartDashboard.putNumber("Right Velocity", getRightVelocity());
    SmartDashboard.putNumber("Gyro", getAngle());
  }
}
