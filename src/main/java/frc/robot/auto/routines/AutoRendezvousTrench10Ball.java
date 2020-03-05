/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.routines;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.auto.TrajectoryGenerator;
import frc.robot.auto.commands.AutoShootMid;
import frc.robot.auto.commands.ResetOdometryAuto;
import frc.robot.auto.commands.StopTrajectory;
import frc.robot.commands.sequences.FeedCells;
import frc.robot.commands.sequences.IndexCells;
import frc.robot.commands.sequences.IntakeCells;
import frc.robot.commands.sequences.ResetHopper;
import frc.robot.commands.sequences.ResetShooter;
import frc.robot.subsystems.Drive;

public class AutoRendezvousTrench10Ball extends SequentialCommandGroup {
        TrajectoryGenerator mTrajectories = TrajectoryGenerator.getInstance();
        Drive mDrive = Drive.getInstance();
        /**
         * Add your docs here.
         */
        public AutoRendezvousTrench10Ball() {
                addCommands(
                        new ResetOdometryAuto(),
                        //Intake In Parallel
                        new ParallelCommandGroup(                        
                                new AutoShootMid(),
                                new IntakeCells()),
                        new RamseteCommand(
                                mTrajectories.getCenterStartToRendezvous2ball(),
                                mDrive::getPose,
                                new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
                                new SimpleMotorFeedforward(Constants.ksVolts,
                                        Constants.kvVoltSecondsPerMeter,
                                        Constants.kaVoltSecondsSquaredPerMeter),
                                Constants.kDriveKinematics,
                                mDrive::getWheelSpeeds,
                                new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel),
                                new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel),
                                // RamseteCommand passes volts to the callback
                                mDrive::tankDriveVolts,
                                mDrive),

                        new StopTrajectory(),
                        new IndexCells(),
                        new RamseteCommand(
                                mTrajectories.getRendezvous2BallToStartOfTrench(),
                                mDrive::getPose,
                                new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
                                new SimpleMotorFeedforward(Constants.ksVolts,
                                        Constants.kvVoltSecondsPerMeter,
                                        Constants.kaVoltSecondsSquaredPerMeter),
                                Constants.kDriveKinematics,
                                mDrive::getWheelSpeeds,
                                new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel),
                                new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel),
                                // RamseteCommand passes volts to the callback
                                mDrive::tankDriveVolts,
                                mDrive),

                        new StopTrajectory(),
                        new ParallelDeadlineGroup(
                                new WaitCommand(2), 
                                new FeedCells()),
                        new ResetHopper(),
                        new ResetShooter(),
                        new IntakeCells(),
                        new RamseteCommand(
                                mTrajectories.getStartOfTrenchToEndOfTrench(),
                                mDrive::getPose,
                                new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
                                new SimpleMotorFeedforward(Constants.ksVolts,
                                        Constants.kvVoltSecondsPerMeter,
                                        Constants.kaVoltSecondsSquaredPerMeter),
                                Constants.kDriveKinematics,
                                mDrive::getWheelSpeeds,
                                new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel),
                                new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel),
                                // RamseteCommand passes volts to the callback
                                mDrive::tankDriveVolts,
                                mDrive),
                        new StopTrajectory(),
                        new WaitCommand(.25),
                        new IndexCells(),
                        new AutoShootMid(),
                        new RamseteCommand(
                                mTrajectories.getEndOfTrenchToStartOfTrench(),
                                mDrive::getPose,
                                new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
                                new SimpleMotorFeedforward(Constants.ksVolts,
                                        Constants.kvVoltSecondsPerMeter,
                                        Constants.kaVoltSecondsSquaredPerMeter),
                                Constants.kDriveKinematics,
                                mDrive::getWheelSpeeds,
                                new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel),
                                new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel),
                                // RamseteCommand passes volts to the callback
                                mDrive::tankDriveVolts,
                                mDrive),
                        new StopTrajectory(),
                        new ParallelDeadlineGroup(
                                new WaitCommand(2.5), 
                                new FeedCells()),
                        new ResetShooter(),
                        new ResetHopper()
                );
        }
}
