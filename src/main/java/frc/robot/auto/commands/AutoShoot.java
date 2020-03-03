/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.Track;

public class AutoShoot extends CommandBase {
  /**
   * Creates a new IndexTime.
   */
  double seconds;
  public AutoShoot(double seconds) {
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
    new Track();
      if (RobotContainer.timer.get() <= seconds) {
        if (RobotContainer.turret.hasTarget()) {
          RobotContainer.shooter.setShooterRPM(RobotContainer.shooter.calculateDesiredOutput(RobotContainer.turret.getTargetArea()));
        }
        else {
          RobotContainer.shooter.setShooterRPM(5000);
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
