/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Hopper.HopperControlMode;

public class AutoIndex extends CommandBase {
  /**
   * Creates a new IndexTime.
   */
  HopperControlMode controlMode;
  double seconds;
  public AutoIndex(HopperControlMode controlMode, double seconds) {
    this.controlMode = controlMode;
    this.seconds = seconds;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.timer.reset();
    RobotContainer.timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if (RobotContainer.timer.get() <= seconds) {
        if (controlMode.equals(HopperControlMode.INTAKING)) {
          RobotContainer.hopper.setControlMode(HopperControlMode.INTAKING);
        }
        else if (controlMode.equals(HopperControlMode.FEEDING)) {
          RobotContainer.hopper.setControlMode(HopperControlMode.FEEDING);
        }
      }
      else {
        end(false);
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.timer.stop();
    RobotContainer.hopper.stopSystems();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
