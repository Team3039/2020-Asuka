package frc.team3039.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.team3039.robot.RobotContainer;

public class ActuateIntake extends CommandBase {

  public ActuateIntake() {
    addRequirements((Subsystem) RobotContainer.intake);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    RobotContainer.intake.deploy();
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.intake.retract();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
