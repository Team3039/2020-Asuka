package frc.team3039.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team3039.robot.RobotContainer;

public class DeployBuddyClimb extends CommandBase {

  public DeployBuddyClimb() {
    addRequirements((Subsystem) RobotContainer.climber);
  }

  @Override
  public void initialize() {
    RobotContainer.climber.extend();
  }

  @Override
  public void execute() { }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
