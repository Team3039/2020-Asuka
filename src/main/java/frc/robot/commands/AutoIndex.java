/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoIndex extends CommandBase {
  /**
   * Creates a new IndexTime.
   */
  double seconds;
  public AutoIndex(double seconds) {
    this.seconds = seconds;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      RobotContainer.timer.reset();
      RobotContainer.timer.start();
      if (RobotContainer.timer.get() <= seconds) {
        RobotContainer.hopper.runBouncer();
        RobotContainer.hopper.runBelts();
      }
      else {
        end(false);
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.timer.stop();
    RobotContainer.hopper.stopBouncer();
    RobotContainer.hopper.stopBelts();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
