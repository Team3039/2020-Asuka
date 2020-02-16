package frc.team3039.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3039.robot.RobotMap;

public class Climber extends SubsystemBase {
  
  public Solenoid climbDeployer = new Solenoid(RobotMap.CLIMB_DEPLOYER);
  public TalonSRX climberA = new TalonSRX(RobotMap.CLIMBER_A);
  public TalonSRX climberB = new TalonSRX(RobotMap.CLIMBER_B);
  public Solenoid buddyDeploy = new Solenoid(RobotMap.BUDDY_DEPLOY);
  
  public Climber() {}

  public void deploy() {
    climbDeployer.set(true);
  }

  public void retract() {
    climberA.set(ControlMode.PercentOutput, -.75);
    climberB.follow(climberA);
  }

  public void stop() {
    climberA.set(ControlMode.PercentOutput, 0);
    climberB.follow(climberA);
  }

  public void extend(){
    buddyDeploy.set(true);
  }
 
 
  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
