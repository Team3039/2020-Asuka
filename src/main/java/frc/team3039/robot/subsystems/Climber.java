package frc.team3039.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3039.robot.AutoRoutineSelector;
import frc.team3039.robot.RobotMap;

public class Climber extends Subsystem {
  private static Climber mInstance = new Climber();


  public Solenoid climbDeployer;
  public TalonSRX climberA;
  public TalonSRX climberB;
  public Solenoid buddyDeploy;
  
  public Climber() {
    climbDeployer = new Solenoid(RobotMap.CLIMB_DEPLOY_PCM_ID);
    climberA = new TalonSRX(RobotMap.CLIMB_MOTOR_A_CAN_ID);
    climberB = new TalonSRX(RobotMap.CLIMB_MOTOR_B_CAN_ID);
    buddyDeploy = new Solenoid(RobotMap.BUDDY_RELEASE_PCM_ID);
  }

  public static Climber getInstance() {
    if (mInstance == null) {
      mInstance = new Climber();
    }
    return mInstance;
  }

  public void deploy() {
    climbDeployer.set(true);
  }

  public void extend(){
    buddyDeploy.set(true);
  }

  public void setSpeed(double speed){
    climberA.set(ControlMode.PercentOutput, speed);
    climberB.follow(climberA);
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

  }

 
  



}
