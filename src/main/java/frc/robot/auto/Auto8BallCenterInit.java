/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autoCommands.DriveLinear;
import frc.robot.autoCommands.Rotate;
import frc.robot.commands.Track;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Auto8BallCenterInit extends SequentialCommandGroup {
  /**
   * Creates a new Auto8BallCenterInit.
   */
  public Auto8BallCenterInit() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new Rotate(45),
      new WaitCommand(.25),
      new DriveLinear(103, 45),
      new WaitCommand(.25),
      new Rotate(36.5),
      new WaitCommand(.25),
      new DriveLinear(120, 36.5),
      new WaitCommand(.25),
      new DriveLinear(-90, 36.5)
    );
  }
}
