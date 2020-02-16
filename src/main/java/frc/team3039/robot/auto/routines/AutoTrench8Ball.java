/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3039.robot.auto.routines;

import frc.team3039.robot.auto.AutoModeEndedException;
import frc.team3039.robot.auto.AutoRoutineBase;
import frc.team3039.robot.auto.actions.DriveTrajectory;
import frc.team3039.robot.auto.actions.SeriesAction;
import frc.team3039.robot.auto.actions.WaitAction;
import frc.team3039.robot.paths.TrajectoryGenerator;
import frc.team3039.robot.subsystems.Drive;

/**
 * Add your docs here.
 */
public class AutoTrench8Ball extends AutoRoutineBase {
    DriveTrajectory first_path;
    DriveTrajectory second_path;
    DriveTrajectory third_path;

    public AutoTrench8Ball() {

        first_path = new DriveTrajectory(
                registerTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().centerStartToEndOfTrench),
                true);

        second_path = new DriveTrajectory(
                registerTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().endOfTrenchToCenterPort),
                false);

        third_path = new DriveTrajectory(
                registerTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().centerStartToEndOfTrench),
                false);

    }

    @Override
    protected void routine() throws AutoModeEndedException {
        runAction(new SeriesAction(new WaitAction(2)));
        runAction(new SeriesAction(first_path));
        runAction(new SeriesAction(new WaitAction(3)));
        runAction(new SeriesAction(second_path));
        runAction(new SeriesAction(new WaitAction(1.5)));
    }
}