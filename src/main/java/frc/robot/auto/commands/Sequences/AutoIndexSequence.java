/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.commands.Sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.commands.AutoIndex;
import frc.robot.subsystems.Hopper.HopperControlMode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoIndexSequence extends SequentialCommandGroup {
  /**
   * Creates a new AutoIndexSequence.
   */
  double seconds;
  public AutoIndexSequence(double seconds) {
    super(
      new WaitCommand(1.5), 
      new AutoIndex(HopperControlMode.FEEDING, seconds)
    );
    this.seconds = seconds;
  }
}
