package frc.team3039.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3039.robot.RobotContainer;

public class Shoot extends CommandBase {

  double RPM;
  public Shoot(double RPM) {
    addRequirements(RobotContainer.shooter);
    this.RPM = RPM;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    RobotContainer.shooter.setShooterRPM(RPM);
  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.shooter.setShooterRPM(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
