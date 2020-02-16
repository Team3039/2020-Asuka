package frc.team3039.robot.commands;

import java.lang.module.ModuleDescriptor.Requires;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3039.robot.RobotContainer;

public class Climb extends CommandBase {

  public Climb() {
    addRequirements(RobotContainer.climber);
  }
 
  @Override
  public void initialize() {
    RobotContainer.climber.retract();
  }

  @Override
  public void execute() {
    RobotContainer.climber.retract();
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
