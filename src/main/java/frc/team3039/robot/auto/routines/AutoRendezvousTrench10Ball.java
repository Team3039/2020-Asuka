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
import frc.team3039.robot.paths.TrajectoryGenerator;

/**
 * Add your docs here.
 */
public class AutoRendezvousTrench10Ball extends AutoRoutineBase {
    DriveTrajectory first_path;
    DriveTrajectory second_path;
    DriveTrajectory third_path;
    DriveTrajectory fourth_path;

    public AutoRendezvousTrench10Ball() {
    first_path = new DriveTrajectory(
            registerTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().centerStartToRendezvous2ball),
            true);

    second_path = new DriveTrajectory(
                registerTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().rendezvous2ballToCloseShot),
                false);


    third_path = new DriveTrajectory(
            registerTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().centerStartToEndOfTrench),
            false);

    fourth_path = new DriveTrajectory(
                registerTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet().endOfTrenchToCenterPort),
                false);
    }


    @Override
    protected void routine() throws AutoModeEndedException {

    }
}
