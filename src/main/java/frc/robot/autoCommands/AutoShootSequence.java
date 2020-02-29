/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autoCommands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.ActuateHood;
import frc.robot.commands.Index;
import frc.robot.commands.Shoot;
import frc.robot.commands.ShootSequence;
import frc.robot.commands.Track;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoShootSequence extends ParallelDeadlineGroup {
  /**
   * Creates a new PrepShooter.
   */
  double RPM;
  double seconds;
  public AutoShootSequence(double seconds, double RPM) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new ActuateHood(), new WaitCommand(.10), new Shoot(RPM)
      );
    this.seconds = seconds;
    this.RPM = RPM;
  }
}
