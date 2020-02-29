/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autoPaths;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.DriveLinear;
import frc.robot.commands.Index;
import frc.robot.commands.ActuateIntake;
import frc.robot.commands.RunIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class InitLeftToTrench extends ParallelDeadlineGroup {
  /**
   * Creates a new InitLineToTrench.
   */
  public InitLeftToTrench() {
    // Add your commands in the super() call.  Add the deadline first.
    super(
        new DriveLinear(120, 45),
        new ActuateIntake(),
        new RunIntake()
    );
  }
}
