/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autoSequences;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.autoCommands.AutoShoot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoShootSequence extends ParallelRaceGroup {
  /**
   * Creates a new AutoShootSequence.
   */
  double seconds;
  public AutoShootSequence(double seconds) {
    super(
      new AutoShoot(seconds), 
      new AutoIndexSequence(seconds)
    );
    this.seconds = seconds;
  }
}
