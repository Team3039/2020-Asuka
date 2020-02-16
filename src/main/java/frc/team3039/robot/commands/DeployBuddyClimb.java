package frc.team3039.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3039.robot.RobotContainer;

public class DeployBuddyClimb extends CommandBase {

  public DeployBuddyClimb() {
    addRequirements(RobotContainer.climber);
  }

  @Override
  public void initialize() {
    RobotContainer.climber.extend();
  }

  @Override
  public void execute() {
    RobotContainer.climber.extend();
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.climber.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
