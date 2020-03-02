/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.ActuateIntake;
import frc.robot.commands.RunIntake;

public class DriveWithIntake extends CommandBase {
  /**
   * Creates a new DriveWithIntake.
   */
  double distance;
  double angle;
  double seconds;
  public DriveWithIntake(double distance, double angle) {
    this.distance = distance;
    this.angle = angle;
    this.seconds = seconds;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.timer.reset();    
    RobotContainer.timer.start();    
    new ActuateIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (RobotContainer.timer.get() <= seconds) {
      new DriveLinear(distance, angle);
      new RunIntake();
    }
    else {
      RobotContainer.drivetrain.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.timer.stop();    

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
