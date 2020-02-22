/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.controllers.PS4Gamepad;

/**
 * The Drivetrain Class contains the wheelbase's motors and all sensor information for the
 * robot's position on the field.
 */
public class Drivetrain extends SubsystemBase {

  private TalonFX leftFrontDrive = new TalonFX(RobotMap.leftFrontDrive);
  private TalonFX leftRearDrive = new TalonFX(RobotMap.leftRearDrive);
  private TalonFX rightFrontDrive = new TalonFX(RobotMap.rightFrontDrive);
  private TalonFX rightRearDrive = new TalonFX(RobotMap.rightRearDrive);

  public Drivetrain() {
    setNeutralMode(NeutralMode.Brake);
    rightFrontDrive.follow(rightRearDrive);
    leftFrontDrive.follow(leftRearDrive);
    leftRearDrive.configSelectedFeedbackCoefficient(DRIVE_PPR_TO_INCHES);
    rightRearDrive.configSelectedFeedbackCoefficient(DRIVE_PPR_TO_INCHES);
  }
  
  public void joystickControl(PS4Gamepad gp) {
    //Tele-Op Driving
    //Each Motor is Set to Brake Mode, the motor speeds are set in an Arcade Drive fashion
    double y = gp.getLeftYAxis() * DRIVER_Y;
    double rot = -1 * gp.getRightXAxis() * DRIVER_ROT;

    //Calculated Outputs (Limits Output to 12V)
    double leftOutput = y + rot;
    double rightOutput = rot - y;

    //Assigns Each Motor's Power
    leftRearDrive.set(ControlMode.PercentOutput, leftOutput);
    rightRearDrive.set(ControlMode.PercentOutput, rightOutput);
  }

  public void stop() {
    leftRearDrive.set(ControlMode.PercentOutput, 0);
    rightRearDrive.set(ControlMode.PercentOutput, 0);   
  }

  public void setNeutralMode(NeutralMode mode) {
    //Set Motor's Neutral/Idle Mode to Brake or Coast
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
    return degrees * Math.PI/180;
  }

  // public void calibrateGyro() {
	// 	gyroPigeon.enterCalibrationMode(CalibrationMode.Temperature, TalonSRXEncoder.TIMEOUT_MS);
	// }

  @Override
  public void periodic() {
    joystickControl(RobotContainer.getDriver());
    setNeutralMode(NeutralMode.Coast);
    // differentialDriveControl(RobotContainer.getDriver());
    SmartDashboard.putNumber("Left Position", getLeftPosition());
    SmartDashboard.putNumber("Right Position", getRightPosition());
    SmartDashboard.putNumber("Left Velocity", getLeftVelocity());
    SmartDashboard.putNumber("Right Velocity", getRightVelocity());
  }

  
}
