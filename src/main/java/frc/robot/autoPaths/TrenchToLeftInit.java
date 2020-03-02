/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autoPaths;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.autoCommands.DriveLinear;
import frc.robot.commands.Shoot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TrenchToLeftInit extends ParallelDeadlineGroup {
  /**
   * Creates a new TrenchToLeftInit.
   */
  public TrenchToLeftInit() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new DriveLinear(-200, 45), new Shoot(5000));
  }
}
