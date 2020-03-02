/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.TURRET_PPR_TO_DEGREES;
import static frc.robot.Constants.kP_TURRET;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.controllers.PS4Gamepad;

/**
 * This device is responisble for the rotational control of the "Shooter" and the tracking of the 
 * retro-reflective target on the "Power Port"
 */
public class Turret extends SubsystemBase {
  
  public TalonSRX turret = new TalonSRX(RobotMap.turret);
  public DigitalInput turretSwitch = new DigitalInput(RobotMap.turretSwitch);

  public static double startPos = -90;
  
  public Turret() {
    turret.setSelectedSensorPosition(0);
    turret.configSelectedFeedbackCoefficient(TURRET_PPR_TO_DEGREES); //Convert to Degrees of Revolution
    turret.setSensorPhase(true);
    setLed(false);
    setPipeline(0);
    setCamMode(false);
    turret.config_kP(0, 0.05);
  }

  public enum TurretControlMode {
    IDLE, 
    TRACKING,
    WALL, 
    POSITION
  }

  public TurretControlMode turretControlMode = TurretControlMode.IDLE;

  public synchronized TurretControlMode getControlMode() {
    return turretControlMode;
  }

  public synchronized void setControlMode(TurretControlMode controlMode) {
    this.turretControlMode = controlMode;
  }

  public void setLed(boolean isOn) {
    if (isOn) {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3); //Force LED on
    }
    else {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1); //Force LED off
    }
  }

  public void setPipeline(int pipeline) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline); //Set the Pipeline #
  }

  public void setCamMode(boolean isTracking) {
    if (isTracking) {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
    }
    else {
      NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
    }
  }

  public void idle() {
    setLed(false);
    setCamMode(false);
  }

  public void aim(TurretControlMode controlMode, double position) {
    double errorX = (getTargetX() - getCurrentPosition()) * kP_TURRET;

    if (getTurretPosition() <= -45 || turretSwitch.get()) {
      turret.set(ControlMode.PercentOutput, .15);
    }
    else if (getTurretPosition() >= 315 || turretSwitch.get()) {
      turret.set(ControlMode.PercentOutput, -.15);
    }
    else if (controlMode.equals(TurretControlMode.WALL)) {
      trackWall();
    }
    else if (controlMode.equals(TurretControlMode.TRACKING)) {
      setTrackingMode();
      // if (onTarget() == false) {
      //   trackWall();
      // }
      // else {
        turret.set(ControlMode.PercentOutput, errorX);
      // }
    }
    else if (controlMode.equals(TurretControlMode.POSITION)) {
      setDriverCamMode();
      setTurretPosition(position);
    }
    else if (controlMode.equals(TurretControlMode.IDLE)) {
      setDriverCamMode();
      turret.set(ControlMode.Position, 0);
    }
  }

  public void aim(TurretControlMode controlMode) {

    if (getTurretSwitch()) {
      turret.set(ControlMode.PercentOutput, 0);
    }
    if (controlMode.equals(TurretControlMode.WALL)) {
      trackWall();
    }
    else if (controlMode.equals(TurretControlMode.TRACKING)) {
      trackTarget();
    }
    else if (controlMode.equals(TurretControlMode.IDLE)) {
      setDriverCamMode();
      turret.set(ControlMode.Position, 0);
    }
  }

  public void trackTarget() {
    double errorX = (0 - getTargetX()) * kP_TURRET;

    if (getTurretSwitch() && getTurretPosition() > 70) {
      turret.set(ControlMode.PercentOutput, -.1);
    }
    else if (getTurretSwitch() && getTurretPosition() < -270) {
      turret.set(ControlMode.PercentOutput, .1);
    }
    else {
      turret.set(ControlMode.PercentOutput, errorX);
    }
  }
  
  public void resetPose() {
    double errorX = (getTurretPosition()) * kP_TURRET;

    if (getTurretSwitch() && getTurretPosition() > 70) {
      turret.set(ControlMode.PercentOutput, -.1);
    }
    else if (getTurretSwitch() && getTurretPosition() < -270) {
      turret.set(ControlMode.PercentOutput, .1);
    }
    else {
      turret.set(ControlMode.PercentOutput, errorX);
    }
  }

  public double getTurretPosition() {
    return getCurrentPosition() % 360;
  }

  public void setTrackingMode() {
    setPipeline(0);
    setLed(true);
    setCamMode(true);
  }

  public void setCamMode() {
    setPipeline(0);
    setLed(false);
    setCamMode(false);
  }

  public void setDriverCamMode() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1); //Turns LED off
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1); //Disable Vision Processing and Doubles Exposure
  }

  public double convertAngle(double angle) {
    if (angle >= 0) {
        return angle;
    }
    else {
        return angle + 360;
    }
}

  public void trackWall() {
      setTurretPosition(getTargetX() - RobotContainer.drivetrain.getAngle());
    }

  public void resetTurretPosition() {
    turret.setSelectedSensorPosition(0);
  }

  public void setTurretPosition(double degrees) {
    double modDegrees = degrees % 360;
    turret.set(ControlMode.Position, modDegrees);
  }

  public double getTargetX() {
    return Robot.targetX; 
  }

  public double getTargetY() {
    return Robot.targetY;
  }

  public double getTargetArea() {
    return Robot.targetArea;
  }

  public double getErrorX() {
    return getTargetX() - getCurrentPosition();
  }

  public boolean hasTarget() {
    if(Robot.targetValid == 1) {
      return true;
    }
    else {
      return false;
    }
  }

  public void manualControl(PS4Gamepad gp) {
    while (getTurretSwitch()) {
      if (turret.getSelectedSensorPosition() <= 0) {
        turret.set(ControlMode.PercentOutput, .15);
      }
      else if (turret.getSelectedSensorPosition() >= 0) {
        turret.set(ControlMode.Position, -.15);
      }
      else {
        double rot = gp.getRightXAxis() * .2;
        turret.set(ControlMode.PercentOutput, rot);
      }
    }
  }

  public Boolean getTurretSwitch() {
    return !turretSwitch.get();
  }

  public double getCurrentPosition() {
    return turret.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    turret.selectProfileSlot(0, 0);

    synchronized (Turret.this) {
      switch (getControlMode()) {
        case IDLE:
          setDriverCamMode();
          resetPose();
          break;
        case TRACKING:
          RobotContainer.turret.setTrackingMode();
          RobotContainer.turret.trackTarget();
          break;
        case WALL:
          System.out.println("Set turret to track wall");
          aim(TurretControlMode.WALL);
          break;
        default:
          System.out.println("Unknown turret control mode");
          break;
      }
    }
  }
}
