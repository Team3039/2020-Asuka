package frc.team3039.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3039.robot.RobotContainer;

public class SetTurretModeNear extends CommandBase {

  public SetTurretModeNear() {
    addRequirements(RobotContainer.turret);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    RobotContainer.turret.setTrackingModeNear();
    RobotContainer.turret.aim();

  }

  @Override
  public void end(boolean interrupted) {
    RobotContainer.turret.setDriverCamMode();
    RobotContainer.turret.turret.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
