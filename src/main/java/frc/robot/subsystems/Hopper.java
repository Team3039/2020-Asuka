/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

/**
 * The Intake delivers "Power Cells" to this subsystem to be transfered to the
 * Shooter. This subsystem also "indexes" said "Power Cells" for rapid shooting
 */
public class Hopper extends SubsystemBase {

  public TalonSRX bouncer = new TalonSRX(RobotMap.bouncer);
  public TalonSRX feederBeltsA = new TalonSRX(RobotMap.feederBeltsA);
  public TalonSRX feederBeltsB = new TalonSRX(RobotMap.feederBeltsB);

  public Hopper() {
    feederBeltsA.setInverted(true);
  }

  public void runBelts() {
    feederBeltsA.set(ControlMode.PercentOutput, .40);
    feederBeltsB.set(ControlMode.PercentOutput, .40);
  }

  public void stopBelts() {
    feederBeltsA.set(ControlMode.PercentOutput, 0);
    feederBeltsB.set(ControlMode.PercentOutput, 0);
  }

  public void runBouncer() {
    bouncer.set(ControlMode.PercentOutput, .40);
  }

  public void stopBouncer() {
    bouncer.set(ControlMode.PercentOutput, 0);
  }


  @Override
  public void periodic() {
  }
}
