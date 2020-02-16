package frc.team3039.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3039.robot.RobotContainer;

public class DeployClimb extends CommandBase {

  public DeployClimb() {
    addRequirements(RobotContainer.climber);
  }
  
  @Override
  public void initialize() {
    RobotContainer.climber.deploy();
  }


  @Override
  public void execute() {
    RobotContainer.climber.deploy();
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
