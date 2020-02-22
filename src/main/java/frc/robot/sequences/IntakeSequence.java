/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sequences;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.ActuateIntake;
import frc.robot.commands.RunIntake;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class IntakeSequence extends ParallelDeadlineGroup {
  /**
   * Creates a new CollectCellsSequence.
   */
    public IntakeSequence(Intake intake) {
      super(new ActuateIntake(), new RunIntake());
    }
}

