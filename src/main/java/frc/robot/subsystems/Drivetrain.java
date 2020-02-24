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
import static frc.robot.Constants.DRIVE_TRACKWIDTH;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  public static PigeonIMU drivePigeon = new PigeonIMU(1);

  public DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(inchesToMeters(DRIVE_TRACKWIDTH));

  private PIDController driveCtrl = new PIDController(0,0,0);
  
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
    leftRearDrive.set(ControlMode.PercentOutput, 0);
    rightRearDrive.set(ControlMode.PercentOutput, 0);
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
    drivePigeon.enterCalibrationMode(CalibrationMode.Temperature, 0);
  }

  public double getAngle() {
    return drivePigeon.getCompassHeading();
  }
  
  public void driveToDistanceRaw(double inches) {
    leftFrontDrive.set(ControlMode.Position,inches);
    rightFrontDrive.set(ControlMode.Position,inches);
  }

  public void pidDrive(double distance) {
    double leftOutput = driveCtrl.calculate(getLeftPosition(), distance);
    double rightOutput = driveCtrl.calculate(getRightPosition(), distance);

    double gyroError = 0 - getAngle();
    double kP = 0.05;
    leftFrontDrive.set(ControlMode.PercentOutput, leftOutput + (gyroError * kP));
    rightFrontDrive.set(ControlMode.PercentOutput, rightOutput - (gyroError * kP));
  }


  public void configDrivePID() {
    leftFrontDrive.config_kP(0, 0);
    rightFrontDrive.config_kP(0, 0);
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

  public void setPIDSlot(Boolean isTurning) {
    if (!isTurning) {
      configDrivePID();
    }
    else {
      configTurnPID();
    }
  }
  
  @Override
  public void periodic() {

    joystickControl(RobotContainer.getDriver());

    SmartDashboard.putNumber("Left Position", getLeftPosition());
    SmartDashboard.putNumber("Right Position", getRightPosition());
    SmartDashboard.putNumber("Left Velocity", getLeftVelocity());
    SmartDashboard.putNumber("Right Velocity", getRightVelocity());
    SmartDashboard.putNumber("Gyro", getAngle());
  }
}
