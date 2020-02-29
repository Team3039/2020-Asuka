/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Climber extends SubsystemBase {
  
  public Solenoid climbDeployer = new Solenoid(RobotMap.climbDeployer);
  public TalonSRX climberA = new TalonSRX(RobotMap.climberA);
  public TalonSRX climberB = new TalonSRX(RobotMap.climberB);
  public Solenoid buddyDeploy = new Solenoid(RobotMap.buddyDeploy);

  public Climber() {
    climberA.setInverted(true);
    climberB.setInverted(true);
    climberB.follow(climberA);
  }

  public enum ClimberMode {
    IDLE,
    EXTENDING,
    CLIMBING,
  }

  public ClimberMode climberMode = ClimberMode.IDLE;

  public synchronized ClimberMode getClimberMode() {
    return climberMode;
  }

  public void deploy() {
    climbDeployer.set(true);
  }

  public void retract(double power) {
    climberA.set(ControlMode.PercentOutput, (power * -1));
  }

  public void extend(double power){
    climberA.set(ControlMode.PercentOutput, Math.abs(power));;
  }

  public void stop() {
    climberA.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    synchronized (Climber.this) {
      switch(getClimberMode()) {
        case IDLE:
          break;
        case EXTENDING:

          break;
        case CLIMBING:
          break;
      }
    }
  }
}
