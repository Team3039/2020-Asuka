/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Turret.TurretControlMode;

public class Track extends CommandBase {
  /**
   * Creates a new Track.
   */
  public Track() {
    addRequirements(RobotContainer.turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // double errorX = (RobotContainer.turret.getTargetX() - RobotContainer.turret.getCurrentPosition()) * Constants.kP_TURRET;

    //   if (RobotContainer.turret.getTurretSwitch()) {
    //     RobotContainer.turret.turret.set(ControlMode.PercentOutput, 0);
    //   }
    //   else {
      RobotContainer.turret.setTrackingMode();
        // RobotContainer.turret.trackTarget();
      // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.turret.setDriverCamMode();
    RobotContainer.turret.turret.set(ControlMode.PercentOutput, 0);
    // RobotContainer.turret.setControlMode(TurretControlMode.IDLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
