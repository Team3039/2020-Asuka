package frc.team3039.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3039.robot.RobotContainer;

public class RunIntake extends CommandBase {

  public RunIntake() {
    addRequirements(RobotContainer.intake);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    RobotContainer.intake.start();
    // RobotContainer.hopper.runRevolver();
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.intake.stop();
    // RobotContainer.hopper.stopRevolver();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
