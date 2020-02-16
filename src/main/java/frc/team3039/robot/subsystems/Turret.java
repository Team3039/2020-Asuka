package frc.team3039.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.team3039.robot.Constants.*;
import frc.team3039.robot.Robot;
import frc.team3039.robot.RobotMap;

/**
 * This device is responisble for the rotational control of the "Shooter" and the tracking of the 
 * retro-reflective target on the "Power Port"
 */
public class Turret extends SubsystemBase {
  
  public TalonSRX turret = new  TalonSRX(RobotMap.TURRET);
  
  public Turret() {
    turret.setSelectedSensorPosition(0);
    turret.setSensorPhase(true);
    setDriverCamMode();
  }

  public void turnOnLED() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3); //Turns LED on
  }

  public void setTrackingModeFar() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(9); //Begin Processing Vision
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0); //Turns LED on
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0); //Disable Vision Processing and Doubles Exposure
  }

  public void setTrackingModeNear() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0); //Begin Processing Vision
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0); //Turns LED on
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0); //Disable Vision Processing and Doubles Exposure
  }

  public void setDriverCamMode() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1); //Turns LED off
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1); //Disable Vision Processing and Doubles Exposure
  }

  public void aim() {
    double kP = kP_TURRET;
    
    double error = getTargetX() * kP;

    if(getCurrentPosition() <= -4000) {
      turret.set(ControlMode.PercentOutput, .15);
    }
    else if(getCurrentPosition() >= 4000) {
      turret.set(ControlMode.PercentOutput, -.15);
    }
    else {
      turret.set(ControlMode.PercentOutput, error);
    }

    SmartDashboard.putNumber("Error", error);
  }

  public double getTargetX() {
    return Robot.targetX; 
  }
  
  public double getCurrentPosition() {
    return turret.getSelectedSensorPosition();        
  }

  public double getTargetY() {
    return Robot.targetY;
  }

  public double getTargetArea() {
    return Robot.targetArea;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret Position", getCurrentPosition());
    SmartDashboard.putNumber("target Y", getTargetY());
    // double errorX = getTargetX() - getCurrentPosition();
    // SmartDashboard.putNumber("Error", errorX);
    // SmartDashboard.putNumber("TargetX", getTargetX());
  }

}
