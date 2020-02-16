package frc.team3039.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team3039.robot.RobotContainer;

public class RunIntake extends CommandBase {

  public RunIntake() {
    addRequirements((Subsystem) RobotContainer.intake);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    RobotContainer.intake.setSpeed(.65);
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.intake.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
