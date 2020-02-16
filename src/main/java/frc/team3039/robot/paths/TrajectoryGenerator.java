package frc.team3039.robot.paths;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import frc.team3039.robot.planners.DriveMotionPlanner;
import frc.team3039.utility.lib.geometry.Pose2d;
import frc.team3039.utility.lib.geometry.Pose2dWithCurvature;
import frc.team3039.utility.lib.geometry.Rotation2d;
import frc.team3039.utility.lib.geometry.Translation2d;
import frc.team3039.utility.lib.trajectory.LazyLoadTrajectory;
import frc.team3039.utility.lib.trajectory.Trajectory;
import frc.team3039.utility.lib.trajectory.timing.CentripetalAccelerationConstraint;
import frc.team3039.utility.lib.trajectory.timing.TimedState;
import frc.team3039.utility.lib.trajectory.timing.TimingConstraint;

//TODO CHANGE ALL VALUES FOR 2020 FIELD/ROBOT

public class TrajectoryGenerator {

        private static final double kFirstPathMaxVoltage = 9.0;
        private static final double kFirstPathMaxAccel = 60.0;
        private static final double kFirstPathMaxVel = 60.0;

        private static final double kMaxVoltage = 10.0;
        private static final double kPathMaxAccel = 140.0;
        private static final double kPathMaxCentripetalAccel = 120.0;
        private static final double kPathMaxVelocity = 144.0;

        private static TrajectoryGenerator mInstance = new TrajectoryGenerator();
        private final DriveMotionPlanner mMotionPlanner;
        private TrajectorySet mTrajectorySet = null;

        public static TrajectoryGenerator getInstance() {
                return mInstance;
        }

        private TrajectoryGenerator() {
                mMotionPlanner = new DriveMotionPlanner();
        }

        public void generateTrajectories() {
                if (mTrajectorySet == null) {
                        System.out.println("Generating trajectories...");
                        mTrajectorySet = new TrajectorySet();
                        System.out.println("Finished trajectory generation");
                }
        }

        public TrajectorySet getTrajectorySet() {
                return mTrajectorySet;
        }

        public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(boolean reversed,
                        final List<Pose2d> waypoints, final List<TimingConstraint<Pose2dWithCurvature>> constraints,
                        double max_vel, // inches/s
                        double max_accel, // inches/s^2
                        double max_voltage) {
                return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, max_vel, max_accel,
                                max_voltage);
        }

        public Trajectory<TimedState<Pose2dWithCurvature>> generateTrajectory(boolean reversed,
                        final List<Pose2d> waypoints, final List<TimingConstraint<Pose2dWithCurvature>> constraints,
                        double start_vel, // inches/s
                        double end_vel, // inches/s
                        double max_vel, // inches/s
                        double max_accel, // inches/s^2
                        double max_voltage) {
                return mMotionPlanner.generateTrajectory(reversed, waypoints, constraints, start_vel, end_vel, max_vel,
                                max_accel, max_voltage);
        }

        public static final Pose2d kStartForward = new Pose2d(136, 67.34, Rotation2d.fromDegrees(0.0));

        public static final Pose2d kMoveForward = new Pose2d(236, 67.34, Rotation2d.fromDegrees(0.0));

        public static final Pose2d kStartReversed = new Pose2d(136, 67.34, Rotation2d.fromDegrees(-180.0));

        public static final Pose2d kMoveReversed = new Pose2d(236, 67.34, Rotation2d.fromDegrees(-180.0));


        // CRITICAL POSES
        // Origin is the center of the robot when the robot is placed against the middle
        // of the alliance station wall.
        // +x is towards the center of the field.
        // +y is to the left.
        // ALL POSES DEFINED FOR THE CASE THAT ROBOT STARTS ON RIGHT! (mirrored about +x
        // axis for LEFT)
        public static final Pose2d kStartCenterPowerPort = new Pose2d(136, 67.34, Rotation2d.fromDegrees(90.0));

        public static final Pose2d kStartStealBallTrench = new Pose2d(136, -133, Rotation2d.fromDegrees(0.0));

        public static final Pose2d kStartLeftPowerPort = new Pose2d(136, 16, Rotation2d.fromDegrees(0.0));

        // Trench Poses
        public static final Pose2d kEndOfTrenchBallsPose = new Pose2d(new Translation2d(370, 133),
                        Rotation2d.fromDegrees(0.0));

        public static final Pose2d kThirdBallTrenchPose = new Pose2d(new Translation2d(312, 133),
                        Rotation2d.fromDegrees(0.0));

        public static final Pose2d kSecondBallTrenchPose = new Pose2d(new Translation2d(276, 133),
                        Rotation2d.fromDegrees(0.0));

        public static final Pose2d kFirstBallTrenchPose = new Pose2d(new Translation2d(240, 133),
                        Rotation2d.fromDegrees(0.0));

        public static final Pose2d kStartOfTrenchPose = new Pose2d(new Translation2d(206.57, 133),
                        Rotation2d.fromDegrees(0.0));

        public static final Pose2d kStealBallsPose = new Pose2d(new Translation2d(240,-133),
                        Rotation2d.fromDegrees(0.0));

        //Shooting Poses
        public static final Pose2d kShootCenterFaceTrenchPose = new Pose2d(new Translation2d(136, 67.34),
                Rotation2d.fromDegrees(90.0));

        public static final Pose2d kShootCenterFaceRendezvousPose = new Pose2d(new Translation2d(136, 67.34),
                Rotation2d.fromDegrees(0.0));

        public static final Pose2d kShootCenterFarPose = new Pose2d(new Translation2d(180,67.34),
                Rotation2d.fromDegrees(-90.0));

        //Multipurpose Poses
        public static final Pose2d kMidFieldPose = new Pose2d(new Translation2d(180,0),
                Rotation2d.fromDegrees(-90.0));

        public static final Pose2d kRendezvousPose = new Pose2d(new Translation2d(223,-11),
                Rotation2d.fromDegrees(20.0));
  
        public static final Pose2d kSafeEndPose = new Pose2d(new Translation2d(60,16),
                Rotation2d.fromDegrees(0.0));

        public class TrajectorySet {

                public final LazyLoadTrajectory centerStartToEndOfTrench;
                public final LazyLoadTrajectory endOfTrenchToCenterPort;

                public final LazyLoadTrajectory stealStartToStealBall;
                public final LazyLoadTrajectory stealBallToFarShot;

                public final LazyLoadTrajectory leftStartToRendezvous;
                public final LazyLoadTrajectory rendezvousToCloseShot;

                public final LazyLoadTrajectory driveStraight;
                public final LazyLoadTrajectory driveStraightReversed;
                
                public final LazyLoadTrajectory leftStartToSafe;


                private TrajectorySet() {
                        centerStartToEndOfTrench = new LazyLoadTrajectory(() -> getCenterStartToEndOfTrench());
                        endOfTrenchToCenterPort = new LazyLoadTrajectory(() -> getEndOfTrenchToCenterPort());

                        stealStartToStealBall = new LazyLoadTrajectory(() -> getStealStartToStealBall());
                        stealBallToFarShot = new LazyLoadTrajectory(() -> getStealBallToFarShotPose());

                        driveStraight = new LazyLoadTrajectory(() -> getLevel2StartDriveStraight());
                        driveStraightReversed = new LazyLoadTrajectory(() -> getLevel2StartDriveStraightReversed());

                        leftStartToRendezvous = new LazyLoadTrajectory(() -> getLeftStartToRendezvous());
                        rendezvousToCloseShot = new LazyLoadTrajectory(() -> getRendezvousToCloseShot());

                        leftStartToSafe = new LazyLoadTrajectory(() -> getLeftStartToSafe());
                }
                private Trajectory<TimedState<Pose2dWithCurvature>> getLevel2StartDriveStraightReversed() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kStartReversed);
                        waypoints.add(kMoveReversed);

                        return generateTrajectory(true, waypoints,
                                Arrays.asList(new CentripetalAccelerationConstraint(kPathMaxCentripetalAccel)),
                                0, kFirstPathMaxVel, kFirstPathMaxVel, kFirstPathMaxAccel,
                                kFirstPathMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getLevel2StartDriveStraight() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kStartForward);
                        waypoints.add(kMoveForward);

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(kPathMaxCentripetalAccel)),
                                        0, kFirstPathMaxVel, kFirstPathMaxVel, kFirstPathMaxAccel,
                                        kFirstPathMaxVoltage);
                }

                //Trench 8 Ball Auto Path Start
                private Trajectory<TimedState<Pose2dWithCurvature>> getCenterStartToEndOfTrench() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kStartCenterPowerPort);
                        waypoints.add(kStartOfTrenchPose);
                        waypoints.add(kEndOfTrenchBallsPose);

                        return generateTrajectory(false, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(kPathMaxCentripetalAccel)),
                                        kPathMaxVelocity, kPathMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getEndOfTrenchToCenterPort() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kEndOfTrenchBallsPose);
                        waypoints.add(kStartOfTrenchPose);
                        waypoints.add(kShootCenterFaceTrenchPose);

                        return generateTrajectory(true, waypoints,
                                        Arrays.asList(new CentripetalAccelerationConstraint(kPathMaxCentripetalAccel)),
                                        kPathMaxVelocity, kPathMaxAccel, kMaxVoltage);
                }
                //Trench 8 Ball Auto Path End

                //Trench Steal Auto Path Start
                private Trajectory<TimedState<Pose2dWithCurvature>> getStealStartToStealBall() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kStartStealBallTrench);
                        waypoints.add(kStealBallsPose);

                        return generateTrajectory(false, waypoints,
                                Arrays.asList(new CentripetalAccelerationConstraint(kPathMaxCentripetalAccel)),
                                kPathMaxVelocity, kPathMaxAccel, kMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getStealBallToFarShotPose() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kStealBallsPose);
                        waypoints.add(kMidFieldPose);
                        waypoints.add(kShootCenterFarPose);

                        return generateTrajectory(true, waypoints,
                                Arrays.asList(new CentripetalAccelerationConstraint(kPathMaxCentripetalAccel)),
                                kPathMaxVelocity, kPathMaxAccel, kMaxVoltage);
                }
                //Trench Steal Auto Path End

                //Rendezvous 6 Ball Auto Path Start
                private Trajectory<TimedState<Pose2dWithCurvature>> getLeftStartToRendezvous() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kStartLeftPowerPort);
                        waypoints.add(kRendezvousPose);

                        return generateTrajectory(false, waypoints,
                                Arrays.asList(new CentripetalAccelerationConstraint(kPathMaxCentripetalAccel)),
                                kFirstPathMaxVel, kFirstPathMaxAccel, kFirstPathMaxVoltage);
                }
                private Trajectory<TimedState<Pose2dWithCurvature>> getRendezvousToCloseShot() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kRendezvousPose);
                        waypoints.add(kShootCenterFaceRendezvousPose);

                        return generateTrajectory(true, waypoints,
                                Arrays.asList(new CentripetalAccelerationConstraint(kPathMaxCentripetalAccel)),
                                kFirstPathMaxVel, kFirstPathMaxAccel, kFirstPathMaxVoltage);
                }

                private Trajectory<TimedState<Pose2dWithCurvature>> getLeftStartToSafe() {
                        List<Pose2d> waypoints = new ArrayList<>();
                        waypoints.add(kStartLeftPowerPort);
                        waypoints.add(kSafeEndPose);

                        return generateTrajectory(false, waypoints,
                                Arrays.asList(new CentripetalAccelerationConstraint(kPathMaxCentripetalAccel)),
                                kFirstPathMaxVel, kFirstPathMaxAccel, kFirstPathMaxVoltage);
                }
        }
}
