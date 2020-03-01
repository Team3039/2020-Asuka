/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.statemachines;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hopper.HopperControlMode;
import frc.robot.subsystems.Intake.IntakeControlMode;

public class Cycler extends SubsystemBase {
  /**
   * Creates a new Intake.
   */
  public Cycler() {
  }

  public enum CycleMode {
    IDLE,
    INTAKING,
    INDEXING,
    UNJAMMING
  }

  public CycleMode cycleMode = CycleMode.IDLE;

  public synchronized CycleMode getCycleMode() {
    return cycleMode;
  }

  public synchronized void setCycleMode(CycleMode cycleMode) {
    this.cycleMode = cycleMode;
  }

  @Override
  public void periodic() {
    synchronized(Cycler.this) {
      switch (getCycleMode()) {
        case IDLE:
          RobotContainer.intake.setControlMode(IntakeControlMode.IDLE);
          RobotContainer.hopper.setControlMode(HopperControlMode.IDLE);    
          break;
        case INTAKING:
          RobotContainer.intake.setControlMode(IntakeControlMode.RUNNING);
          RobotContainer.hopper.setControlMode(HopperControlMode.INTAKING);    
          break;
        case INDEXING:        
          RobotContainer.hopper.setControlMode(HopperControlMode.INDEXING);  
          RobotContainer.intake.setControlMode(IntakeControlMode.RUNNING);
          break;
        case UNJAMMING:
          RobotContainer.hopper.setControlMode(HopperControlMode.UNJAMMING);  
          break;
      }
    }
  }
}
