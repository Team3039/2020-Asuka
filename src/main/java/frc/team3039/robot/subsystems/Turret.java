package frc.team3039.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.team3039.robot.Constants.*;

import frc.team3039.robot.AutoRoutineSelector;
import frc.team3039.robot.Robot;
import frc.team3039.robot.RobotMap;
import frc.team3039.robot.loops.ILooper;
import frc.team3039.robot.loops.Loop;

/**
 * This device is responisble for the rotational control of the "Shooter" and the tracking of the 
 * retro-reflective target on the "Power Port"
 */
public class Turret extends Subsystem {
  private static Turret mInstance = new Turret();


  public TalonSRX turret = new  TalonSRX(RobotMap.TURRET_MOTOR_CAN_ID);

  public enum TurretControlMode {
    OPEN_LOOP,
    TRACKING,
    POSITION,
  }

  public TurretControlMode mTurretControlMode = TurretControlMode.OPEN_LOOP;

  private final Loop mLoop = new Loop() {

    @Override
    public void onStart(double timestamp) {

    }

    @Override
    public void onStop(double timestamp) {
    }

    @Override
    public void onLoop(double timestamp) {
      synchronized (Turret.this) {
        switch (getControlMode()) {
          case OPEN_LOOP:
            break;
          case TRACKING:
            aim();
            break;
          case POSITION:
            System.out.println("Set turret to specific angle");
            break;
          default:
            System.out.println("Unknown turret control mode");
            break;
        }
      }
    }
  };

    @Override
    public void registerEnabledLoops(ILooper in) {
      in.register(mLoop);
    }

  public synchronized TurretControlMode getControlMode() {
    return mTurretControlMode;
  }

  public synchronized void setControlMode(TurretControlMode controlMode) {
    this.mTurretControlMode = controlMode;
  }
  
  public Turret() {
    turret = new TalonSRX(RobotMap.TURRET_MOTOR_CAN_ID);

    turret.setSensorPhase(true);
  }

  public static Turret getInstance() {
    if (mInstance == null) {
      mInstance = new Turret();
    }
    return mInstance;
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

  public void setAngle(int angle){
    //Make a method that sets turret angle
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
  public void zeroSensors() {
    turret.setSelectedSensorPosition(0,0,0);
  }

  @Override
  public void stop() {

  }

  @Override
  public boolean checkSystem() {
    return false;
  }


  @Override
  public void outputTelemetry(AutoRoutineSelector.DesiredMode mode) {
    if (mode == AutoRoutineSelector.DesiredMode.TEST) {
      try {
        SmartDashboard.putNumber("Turret Position", getCurrentPosition());
        SmartDashboard.putNumber("target Y", getTargetY());
        double errorX = getTargetX() - getCurrentPosition();
        SmartDashboard.putNumber("Error", errorX);
        SmartDashboard.putNumber("TargetX", getTargetX());
      } catch (Exception e) {
        System.out.println("Desired Mode Error in Turret Subsystem");
      }
    } else if (mode == AutoRoutineSelector.DesiredMode.COMPETITION) {

    }
  }
}
