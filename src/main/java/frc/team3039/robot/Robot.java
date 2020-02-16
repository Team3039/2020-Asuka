package frc.team3039.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.team3039.robot.AutoRoutineSelector.DesiredMode;
import frc.team3039.robot.auto.AutoModeExecutor;
import frc.team3039.robot.auto.AutoRoutineBase;
import frc.team3039.robot.loops.Looper;
import frc.team3039.robot.paths.TrajectoryGenerator;
import frc.team3039.robot.subsystems.*;
import frc.team3039.utility.CrashTracker;
import frc.team3039.utility.lib.geometry.Pose2d;

import java.util.Arrays;
import java.util.Optional;

public class Robot extends TimedRobot {
	RobotContainer mRobotContainer;

   //Vision Information
   public static double targetValid; //Whether the limelight has any valid targets (0 or 1)
   public static double targetX; //Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
   public static double targetY; //Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
   public static double targetArea; //Target Area (0% of image to 100% of image)
   public static double targetSkew; //Skew or rotation (-90 degrees to 0 degrees)


	private Looper mEnabledLooper = new Looper();
	private Looper mDisabledLooper = new Looper();

	private TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
	private AutoRoutineSelector mAutoRoutineSelector = new AutoRoutineSelector();
	private AutoModeExecutor mAutoModeExecutor;
	private DesiredMode mOperationMode;

	private final SubsystemManager mSubsystemManager = new SubsystemManager(
			Arrays.asList(
					RobotStateEstimator.getInstance(),
					Drive.getInstance(),
					Hopper.getInstance(),
					Shooter.getInstance(),
					Turret.getInstance(),
					ControlPanel.getInstance(),
					Intake.getInstance(),
					Climber.getInstance()

			)
	);

	// Declare subsystems
	public static final Drive mDrive = Drive.getInstance();
	public static final RobotStateEstimator mRobotStateEstimator = RobotStateEstimator.getInstance();
	private RobotState mRobotState = RobotState.getInstance();

	public void zeroAllSensors() {
		mSubsystemManager.zeroSensors();
	}

	public Robot(){
		CrashTracker.logRobotConstruction();
	}

	// Called at the start of connection
	@Override
	public void robotInit() {
		try {
			mRobotContainer = new RobotContainer();

			CrashTracker.logRobotInit();

			mSubsystemManager.registerEnabledLoops(mEnabledLooper);
			mSubsystemManager.registerDisabledLoops(mDisabledLooper);

			mTrajectoryGenerator.generateTrajectories();

			mAutoRoutineSelector.updateModeCreator();

			zeroAllSensors();

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	// Called every loop for all modes
	public void robotPeriodic() {
		outputToSmartDashboard();
		CommandScheduler.getInstance().run();
	}

	// Called once when is disabled
	@Override
	public void disabledInit() {
		SmartDashboard.putString("Match Cycle", "DISABLED");

		try {
			CrashTracker.logDisabledInit();
			mEnabledLooper.stop();
			if (mAutoModeExecutor != null) {
				mAutoModeExecutor.stop();
			}

			zeroAllSensors();
			RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

			// Reset all auto mode state.
			mAutoRoutineSelector.reset();
			mAutoRoutineSelector.updateModeCreator();
			mAutoModeExecutor = new AutoModeExecutor();

			mDisabledLooper.start();

			zeroAllSensors();

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	// Called constantly when the robot is disabled
	@Override
	public void disabledPeriodic() {
		SmartDashboard.putString("Match Cycle", "DISABLED");

		try {
			outputToSmartDashboard();
			mOperationMode = mAutoRoutineSelector.getDesiredMode();
			mAutoRoutineSelector.updateModeCreator();

			Optional<AutoRoutineBase> autoMode = mAutoRoutineSelector.getAutoMode();
			if (autoMode.isPresent() && autoMode.get() != mAutoModeExecutor.getAutoMode()) {
				System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
				mAutoModeExecutor.setAutoMode(autoMode.get());
			}
			System.gc();
		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

// Called once at the start of auto
	@Override
	public void autonomousInit() {
		SmartDashboard.putString("Match Cycle", "AUTONOMOUS");

		try {
			CrashTracker.logAutoInit();
			mDisabledLooper.stop();

			RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

			mDrive.zeroSensors();

			mAutoModeExecutor.start();

			mEnabledLooper.start();

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	// Called constantly through autonomous
	@Override
	public void autonomousPeriodic() {
		SmartDashboard.putString("Match Cycle", "AUTONOMOUS");
		outputToSmartDashboard();
		try {
			System.out.println("Error in Autonomous Periodic");

		} catch (Throwable t) {
			CrashTracker.logThrowableCrash(t);
			throw t;
		}

	}

	// Called once at the start of teleOp
	@Override
	public void teleopInit() {
			if (mAutoModeExecutor != null) {
				mAutoModeExecutor.stop();
			}

		RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
		mEnabledLooper.start();

		mDrive.setControlMode(Drive.DriveControlMode.JOYSTICK);

		mDrive.endGyroCalibration();

	}

	// Called constantly through teleOp
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();

		try {
			outputToSmartDashboard();

		}catch (Throwable t){
			CrashTracker.logThrowableCrash(t);
			throw t;
		}
	}

	public double getMatchTime() {
		return m_ds.getMatchTime();
	}

	public void outputToSmartDashboard() {
		mDrive.outputTelemetry(mOperationMode);
		mRobotStateEstimator.outputTelemetry(mOperationMode);
		mAutoRoutineSelector.outputTelemetry();
		mRobotState.outputToSmartDashboard();
		targetValid = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
		targetX = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
		targetY = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
		targetArea = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
		targetSkew = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);
	}
}

