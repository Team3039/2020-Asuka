package frc.team3039.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3039.robot.AutoRoutineSelector.DesiredMode;
import frc.team3039.robot.Constants;
import frc.team3039.robot.RobotContainer;
import frc.team3039.robot.RobotMap;
import frc.team3039.robot.RobotState;
import frc.team3039.robot.loops.Loop;
import frc.team3039.robot.planners.DriveMotionPlanner;
import frc.team3039.utility.DriveSignal;
import frc.team3039.utility.ReflectingCSVWriter;
import frc.team3039.utility.lib.drivers.TalonFXFactory;
import frc.team3039.utility.lib.drivers.TalonSRXEncoder;
import frc.team3039.utility.lib.drivers.TalonSRXFactory;
import frc.team3039.utility.lib.geometry.Pose2d;
import frc.team3039.utility.lib.geometry.Pose2dWithCurvature;
import frc.team3039.utility.lib.geometry.Rotation2d;
import frc.team3039.utility.lib.trajectory.TrajectoryIterator;
import frc.team3039.utility.lib.trajectory.timing.TimedState;

public class Drive extends Subsystem implements Loop{
	private static Drive mInstance = new Drive();

	public enum DriveControlMode {
		JOYSTICK, PATH_FOLLOWING, OPEN_LOOP
	}

	private static final int kVelocityControlSlot = 0;
	private static final double DRIVE_ENCODER_PPR = 2048.0;
//	private static final double ENCODER_TICKS_TO_INCHES = DRIVE_ENCODER_PPR/
//			(Constants.kDriveWheelDiameterInches * Math.PI);

	// Motor Controller Setup
	private TalonFX mLeftMaster, mRightMaster, mLeftSlave, mRightSlave;
	private TalonSRX mPigeonTalon;

	private DriveControlMode mDriveControlMode = DriveControlMode.JOYSTICK;

	// Pigeon Setup
	private PigeonIMU gyroPigeon;
	private double[] yprPigeon = new double[3];
	private short[] xyzPigeon = new short[3];
	private boolean isCalibrating = false;
	private double gyroOffsetDeg = 0;
	protected Rotation2d mAngleAdjustment = Rotation2d.identity();

	// Hardware states
	private PeriodicIO mPeriodicIO;
	private ReflectingCSVWriter<PeriodicIO> mCSVWriter = null;
	private DriveMotionPlanner mMotionPlanner;
	private Rotation2d mGyroOffset = Rotation2d.identity();
	private boolean mOverrideTrajectory = false;
	private boolean isFinished;

		@Override
		public void onStart(double timestamp) {
			synchronized (Drive.this) {
				setBrakeMode(false);
			}
		}

		@Override
		public void onStop(double timestamp) {
		}

		@Override
		public void onLoop(double timestamp) {
			synchronized (Drive.this) {
				DriveControlMode currentControlMode = getControlMode();
				if (currentControlMode == DriveControlMode.JOYSTICK) {
					driveWithJoysticks();
				} else if (!isFinished()) {
//				readPeriodicInputs();
					switch (currentControlMode) {
						case PATH_FOLLOWING:
							updatePathFollower();
//					writePeriodicOutputs();
							break;
						case OPEN_LOOP:
//					writePeriodicOutputs();
							break;
						default:
							System.out.println("Unknown drive control mode: " + currentControlMode);
							break;
					}
				} else {
					// hold in current state
				}
			}
		}

	private void configureMaster(TalonFX talon, boolean left) { //TODO
		talon.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 5, 100);
		final ErrorCode sensorPresent = talon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 100);
		if (sensorPresent != ErrorCode.OK) {
			DriverStation.reportError("Could not detect " + (left ? "left" : "right") + " encoder: " + sensorPresent,
					false);
		}
		talon.enableVoltageCompensation(true);
		talon.configVoltageCompSaturation(12.0, Constants.kLongCANTimeoutMs);
		talon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_50Ms, Constants.kLongCANTimeoutMs);
		talon.configVelocityMeasurementWindow(1, Constants.kLongCANTimeoutMs);
		talon.configClosedloopRamp(Constants.kDriveVoltageRampRate, Constants.kLongCANTimeoutMs);
		talon.configNeutralDeadband(0.04, 0);
	}

	private Drive() {
		try {
			mPeriodicIO = new PeriodicIO();

			mLeftMaster =  TalonFXFactory.createDefaultTalon(RobotMap.DRIVETRAIN_LEFT_MOTOR1_CAN_ID);
			configureMaster(mLeftMaster, true);
			mLeftMaster.setInverted(TalonFXInvertType.Clockwise);

			mRightMaster = TalonFXFactory.createDefaultTalon(RobotMap.DRIVETRAIN_RIGHT_MOTOR1_CAN_ID);
			configureMaster(mRightMaster, false);
			mRightMaster.setInverted(TalonFXInvertType.CounterClockwise);

			mLeftSlave = TalonFXFactory.createPermanentSlaveTalon(RobotMap.DRIVETRAIN_LEFT_MOTOR2_CAN_ID,
					RobotMap.DRIVETRAIN_LEFT_MOTOR1_CAN_ID);
			mLeftSlave.setInverted(InvertType.FollowMaster);

			mRightSlave = TalonFXFactory.createPermanentSlaveTalon(RobotMap.DRIVETRAIN_RIGHT_MOTOR2_CAN_ID,
					RobotMap.DRIVETRAIN_RIGHT_MOTOR1_CAN_ID);
			mRightSlave.setInverted(InvertType.FollowMaster);

			mPigeonTalon = TalonSRXFactory.createPermanentSlaveTalon(RobotMap.PIGEON_IMU_TALON,
					RobotMap.DRIVETRAIN_RIGHT_MOTOR1_CAN_ID);


			reloadGains();

			gyroPigeon = new PigeonIMU(mPigeonTalon);
			gyroPigeon.configFactoryDefault();
			mRightSlave.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 10, 10);

			mMotionPlanner = new DriveMotionPlanner();

            //Force Message
			setBrakeMode(true);
			setBrakeMode(false);

		} catch (Exception e) {
			System.err.println("An error occurred in the Drive constructor");
		}
	}

	public static Drive getInstance() {
		if (mInstance == null) {
			mInstance = new Drive();
		}
		return mInstance;
	}

	// Encoder and Gryo Setup
	private static double rotationsToInches(double rotations) {
		return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
	}

	private static double rpmToInchesPerSecond(double rpm) {
		return rotationsToInches(rpm) / 60;
	}

	private static double inchesToRotations(double inches) {
		return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
	}

	private static double inchesPerSecondToRpm(double inches_per_second) {
		return inchesToRotations(inches_per_second) * 60;
	}

	private static double radiansPerSecondToTicksPer100ms(double rad_s) {
		return rad_s / (Math.PI * 2.0) * 4096.0 / 10.0;
	}

	public double getLeftEncoderRotations() {
		return mPeriodicIO.left_position_ticks / DRIVE_ENCODER_PPR;
	}

	public double getRightEncoderRotations() {
		return mPeriodicIO.right_position_ticks / DRIVE_ENCODER_PPR;
	}

	public double getLeftEncoderDistance() {
		return rotationsToInches(getLeftEncoderRotations());
	}

	public double getRightEncoderDistance() {
		return rotationsToInches(getRightEncoderRotations());
	}

	public double getRightVelocityNativeUnits() {
		return mPeriodicIO.right_velocity_ticks_per_100ms;
	}

	public double getRightLinearVelocity() {
		return rotationsToInches(getRightVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR);
	}

	public double getLeftVelocityNativeUnits() {
		return mPeriodicIO.left_velocity_ticks_per_100ms;
	}

	public double getLeftLinearVelocity() {
		return rotationsToInches(getLeftVelocityNativeUnits() * 10.0 / DRIVE_ENCODER_PPR);
	}

	public double getLinearVelocity() {
		return (getLeftLinearVelocity() + getRightLinearVelocity()) / 2.0;
	}

	public double getAngularVelocity() {
		return (getRightLinearVelocity() - getLeftLinearVelocity()) / Constants.kDriveWheelTrackWidthInches;
	}

	public synchronized void resetEncoders() {
		mRightMaster.setSelectedSensorPosition(0, 0, 0);
		mLeftMaster.setSelectedSensorPosition(0, 0, 0);
		mPeriodicIO = new PeriodicIO();
	}

	public void calibrateGyro() {
		gyroPigeon.enterCalibrationMode(CalibrationMode.Temperature, TalonSRXEncoder.TIMEOUT_MS);
	}

	public void endGyroCalibration() {
		if (isCalibrating) {
			isCalibrating = false;
		}
	}

	public void setGyroOffset(double offsetDeg) {
		gyroOffsetDeg = offsetDeg;
	}

	public synchronized Rotation2d getGyroAngle() {
		return mAngleAdjustment.rotateBy(Rotation2d.fromDegrees(-getGyroAngleDeg()));
	}

	public synchronized void setGyroAngle(Rotation2d adjustment) {
		resetGyro();
		mAngleAdjustment = adjustment;
	}

	public synchronized double getGyroAngleDeg() {
		gyroPigeon.getYawPitchRoll(yprPigeon);
		return -yprPigeon[0] + gyroOffsetDeg;
	}

	public synchronized double getGyroPitchAngle() {
		gyroPigeon.getYawPitchRoll(yprPigeon);
		return yprPigeon[2];
	}

	public short getGyroXAccel() {
		gyroPigeon.getBiasedAccelerometer(xyzPigeon);
		return xyzPigeon[0];
	}

	public short getGyroYAccel() {
		gyroPigeon.getBiasedAccelerometer(xyzPigeon);
		return xyzPigeon[1];
	}

	public short getGyroZAccel() {
		gyroPigeon.getBiasedAccelerometer(xyzPigeon);
		return xyzPigeon[2];
	}

	public synchronized Rotation2d getHeading() {
		return mPeriodicIO.gyro_heading;
	}

	public synchronized void setHeading(Rotation2d heading) {
		System.out.println("SET HEADING: " + heading.getDegrees());

		mGyroOffset = heading.rotateBy(Rotation2d.fromDegrees(gyroPigeon.getFusedHeading()).inverse());
		System.out.println("Gyro offset: " + mGyroOffset.getDegrees());

		mPeriodicIO.gyro_heading = heading;
	}

	public synchronized void resetGyro() {
		gyroPigeon.setYaw(0, TalonSRXEncoder.TIMEOUT_MS);
		gyroPigeon.setFusedHeading(0, TalonSRXEncoder.TIMEOUT_MS);
		setHeading(Rotation2d.identity());

	}

	public void setSpeed(double speed) {
		mRightMaster.set(ControlMode.PercentOutput, speed);
		mLeftMaster.set(ControlMode.PercentOutput, speed);
	}

	@Override
	public void zeroSensors() {
		resetEncoders();
		resetGyro();
	}
	// End

	public synchronized void setOpenLoop(DriveSignal signal) {
		if (mDriveControlMode != DriveControlMode.OPEN_LOOP) {
			setBrakeMode(false);

			System.out.println("Switching to open loop");
			System.out.println(signal);
			mDriveControlMode = DriveControlMode.OPEN_LOOP;
			mRightMaster.configNeutralDeadband(0.04, 0);
			mLeftMaster.configNeutralDeadband(0.04, 0);
		}
		mPeriodicIO.left_demand = signal.getLeft();
		mPeriodicIO.right_demand = signal.getRight();
		mPeriodicIO.left_feedforward = 0.0;
		mPeriodicIO.right_feedforward = 0.0;
	}

	/**
	 * Configures talons for velocity control
	 */
	public synchronized void setVelocity(DriveSignal signal, DriveSignal feedforward) {
		if (mDriveControlMode != DriveControlMode.PATH_FOLLOWING) {
			setBrakeMode(true);
			mLeftMaster.selectProfileSlot(kVelocityControlSlot, 0);
			mRightMaster.selectProfileSlot(kVelocityControlSlot, 0);
			mLeftMaster.configNeutralDeadband(0.0, 0);
			mRightMaster.configNeutralDeadband(0.0, 0);

			setControlMode(DriveControlMode.PATH_FOLLOWING);

		}

		mPeriodicIO.left_demand = signal.getLeft();
		mPeriodicIO.right_demand = signal.getRight();
		mPeriodicIO.left_feedforward = feedforward.getLeft();
		mPeriodicIO.right_feedforward = feedforward.getRight();
	}

	public synchronized void setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>> trajectory) {
		if (mMotionPlanner != null) {
			mOverrideTrajectory = false;
			mMotionPlanner.reset();
			mMotionPlanner.setTrajectory(trajectory);

			setControlMode(DriveControlMode.PATH_FOLLOWING);
		}
	}

	public void overrideTrajectory(boolean value) {
		mOverrideTrajectory = value;
	}

	public boolean isDoneWithTrajectory() {
		if (mMotionPlanner == null) {
			return false;
		}
		return mMotionPlanner.isDone() || mOverrideTrajectory || mDriveControlMode != DriveControlMode.PATH_FOLLOWING;
	}

	private void updatePathFollower() {
		if (mDriveControlMode == DriveControlMode.PATH_FOLLOWING) {
			final double now = Timer.getFPGATimestamp();

			DriveMotionPlanner.Output output = mMotionPlanner.update(now,
					RobotState.getInstance().getFieldToVehicle(now));

			// DriveSignal signal = new DriveSignal(demand.left_feedforward_voltage / 12.0,
			// demand.right_feedforward_voltage / 12.0);

			mPeriodicIO.error = mMotionPlanner.error();
			mPeriodicIO.path_setpoint = mMotionPlanner.setpoint();

			if (!mOverrideTrajectory) {
				setVelocity(
						new DriveSignal(radiansPerSecondToTicksPer100ms(output.left_velocity),
								radiansPerSecondToTicksPer100ms(output.right_velocity)),
						new DriveSignal(output.left_feedforward_voltage / 12.0,
								output.right_feedforward_voltage / 12.0));

				mPeriodicIO.left_accel = radiansPerSecondToTicksPer100ms(output.left_accel) / 1000.0;
				mPeriodicIO.right_accel = radiansPerSecondToTicksPer100ms(output.right_accel) / 1000.0;
			} else {
				setVelocity(DriveSignal.BRAKE, DriveSignal.BRAKE);
				mPeriodicIO.left_accel = mPeriodicIO.right_accel = 0.0;
			}
		} else {
			DriverStation.reportError("Drive is not in path following state", false);
		}
	}

	public synchronized void reloadGains() {
		mLeftMaster.config_kP(kVelocityControlSlot, Constants.kDriveVelocityKp, Constants.kLongCANTimeoutMs);
		mLeftMaster.config_kI(kVelocityControlSlot, Constants.kDriveVelocityKi, Constants.kLongCANTimeoutMs);
		mLeftMaster.config_kD(kVelocityControlSlot, Constants.kDriveVelocityKd, Constants.kLongCANTimeoutMs);
		mLeftMaster.config_kF(kVelocityControlSlot, Constants.kDriveVelocityKf, Constants.kLongCANTimeoutMs);
		mLeftMaster.config_IntegralZone(kVelocityControlSlot, Constants.kDriveVelocityIZone,
				Constants.kLongCANTimeoutMs);

		mRightMaster.config_kP(kVelocityControlSlot, Constants.kDriveVelocityKp, Constants.kLongCANTimeoutMs);
		mRightMaster.config_kI(kVelocityControlSlot, Constants.kDriveVelocityKi, Constants.kLongCANTimeoutMs);
		mRightMaster.config_kD(kVelocityControlSlot, Constants.kDriveVelocityKd, Constants.kLongCANTimeoutMs);
		mRightMaster.config_kF(kVelocityControlSlot, Constants.kDriveVelocityKf, Constants.kLongCANTimeoutMs);
		mRightMaster.config_IntegralZone(kVelocityControlSlot, Constants.kDriveVelocityIZone,
				Constants.kLongCANTimeoutMs);
	}

	@Override
	public void writeToLog() {
	}

	@Override
	public synchronized void readPeriodicInputs() {
		double prevLeftTicks = mPeriodicIO.left_position_ticks;
		double prevRightTicks = mPeriodicIO.right_position_ticks;
		mPeriodicIO.left_position_ticks = mLeftMaster.getSelectedSensorPosition(0);
		mPeriodicIO.right_position_ticks = mRightMaster.getSelectedSensorPosition(0);
		mPeriodicIO.left_velocity_ticks_per_100ms = mLeftMaster.getSelectedSensorVelocity(0);
		mPeriodicIO.right_velocity_ticks_per_100ms = mRightMaster.getSelectedSensorVelocity(0);
		mPeriodicIO.gyro_heading = Rotation2d.fromDegrees(gyroPigeon.getFusedHeading()).rotateBy(mGyroOffset);

		double deltaLeftTicks = ((mPeriodicIO.left_position_ticks - prevLeftTicks) / 4096.0) * Math.PI;
		if (deltaLeftTicks > 0.0) {
			mPeriodicIO.left_distance += deltaLeftTicks * Constants.kDriveWheelDiameterInches;
		} else {
			mPeriodicIO.left_distance += deltaLeftTicks * Constants.kDriveWheelDiameterInches;
		}

		double deltaRightTicks = ((mPeriodicIO.right_position_ticks - prevRightTicks) / 4096.0) * Math.PI;
		if (deltaRightTicks > 0.0) {
			mPeriodicIO.right_distance += deltaRightTicks * Constants.kDriveWheelDiameterInches;
		} else {
			mPeriodicIO.right_distance += deltaRightTicks * Constants.kDriveWheelDiameterInches;

		}

		if (mCSVWriter != null) {
			mCSVWriter.add(mPeriodicIO);
		}

		// System.out.println("control state: " + mDriveControlState + ", left: " +
		// mPeriodicIO.left_demand + ", right: " + mPeriodicIO.right_demand);
	}

	@Override
	public synchronized void writePeriodicOutputs() {
		if (mDriveControlMode == DriveControlMode.OPEN_LOOP) {
			mLeftMaster.set(ControlMode.PercentOutput, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward, 0.0);
			mRightMaster.set(ControlMode.PercentOutput, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward, 0.0);
		} else {
			mLeftMaster.set(ControlMode.Velocity, mPeriodicIO.left_demand, DemandType.ArbitraryFeedForward,
					mPeriodicIO.left_feedforward + Constants.kDriveVelocityKd * mPeriodicIO.left_accel / 1023.0);
			mRightMaster.set(ControlMode.Velocity, mPeriodicIO.right_demand, DemandType.ArbitraryFeedForward,
					mPeriodicIO.right_feedforward + Constants.kDriveVelocityKd * mPeriodicIO.right_accel / 1023.0);
		}
	}
	// End

	// Drive
	public synchronized DriveControlMode getControlMode() {
		return mDriveControlMode;
	}

	public synchronized void setControlMode(DriveControlMode controlMode) {
		this.mDriveControlMode = controlMode;
		setFinished(false);
	}


	public synchronized void driveWithJoysticks() {
		double STICK_DEADZONE = .05;
		double y = -1 * RobotContainer.getInstance().getDriverController().getLeftYAxis() * .9;
		double rot =  -1 * RobotContainer.getInstance().getDriverController().getRightXAxis() * .30;

		//Calculated Outputs (Limits Output to 12V)12312312312312312312312312312311123
		double leftOutput = y - rot;
		double rightOutput = rot + y;

		//Assigns Each Motor's Power
		if(Math.abs(RobotContainer.getInstance().getDriverController().getLeftYAxis()) > STICK_DEADZONE ||
				Math.abs(RobotContainer.getInstance().getDriverController().getRightXAxis()) > STICK_DEADZONE) {
			mLeftMaster.set(ControlMode.PercentOutput, leftOutput);
			mRightMaster.set(ControlMode.PercentOutput, rightOutput);
		}else{
			mLeftMaster.set(ControlMode.PercentOutput, 0.0);
			mRightMaster.set(ControlMode.PercentOutput, 0.0);
		}
	}

	public synchronized  void setBrakeMode(boolean setBrake) {
        if (setBrake) {
            System.out.println("Brake Mode");
            mRightMaster.setNeutralMode(NeutralMode.Brake);
            mRightSlave.setNeutralMode(NeutralMode.Brake);
            mLeftMaster.setNeutralMode(NeutralMode.Brake);
            mLeftSlave.setNeutralMode(NeutralMode.Brake);
        } else {
            System.out.println("Coast Mode");
            mRightMaster.setNeutralMode(NeutralMode.Coast);
            mRightSlave.setNeutralMode(NeutralMode.Coast);
            mLeftMaster.setNeutralMode(NeutralMode.Coast);
            mLeftSlave.setNeutralMode(NeutralMode.Coast);
        }
    }

	public synchronized boolean isFinished() {
		return isFinished;
	}

	public synchronized void setFinished(boolean isFinished) {
		this.isFinished = isFinished;
	}
	// End

	@Override
	public boolean checkSystem() {
		return false;
	}

	public synchronized void startLogging() {
		if (mCSVWriter == null) {
			mCSVWriter = new ReflectingCSVWriter<>("/home/lvuser/DRIVE-LOGS.csv", PeriodicIO.class);
		}
	}

	public synchronized void stopLogging() {
		if (mCSVWriter != null) {
			mCSVWriter.flush();
			mCSVWriter = null;
		}
	}

	@Override
	public synchronized void stop() {
		setOpenLoop(DriveSignal.NEUTRAL);
	}

	@Override
	public void outputTelemetry(DesiredMode operationMode) {
		if (operationMode == DesiredMode.TEST) {
			try {
				SmartDashboard.putNumber("Right Drive Distance", mPeriodicIO.right_distance);
				SmartDashboard.putNumber("Right Drive Ticks", mPeriodicIO.right_position_ticks);
				SmartDashboard.putNumber("Left Drive Ticks", mPeriodicIO.left_position_ticks);
				SmartDashboard.putNumber("Left Drive Distance", mPeriodicIO.left_distance);
				SmartDashboard.putNumber("Right Linear Velocity", getRightLinearVelocity());
				SmartDashboard.putNumber("Left Linear Velocity", getLeftLinearVelocity());

				SmartDashboard.putNumber("x err", mPeriodicIO.error.getTranslation().x());
				SmartDashboard.putNumber("y err", mPeriodicIO.error.getTranslation().y());
				SmartDashboard.putNumber("theta err", mPeriodicIO.error.getRotation().getDegrees());

			} catch (Exception e) {
				System.out.println("Desired Mode Error");
			}
		} else if (operationMode == DesiredMode.COMPETITION) {

			if (getHeading() != null) {
				SmartDashboard.putNumber("Gyro Heading", getHeading().getDegrees());
			}

		}
	}

	public static class PeriodicIO {
		// INPUTS
		public int left_position_ticks;
		public int right_position_ticks;
		public double left_distance;
		public double right_distance;
		public int left_velocity_ticks_per_100ms;
		public int right_velocity_ticks_per_100ms;
		public Rotation2d gyro_heading = Rotation2d.identity();
		public Pose2d error = Pose2d.identity();

		// OUTPUTS
		public double left_demand;
		public double right_demand;
		public double left_accel;
		public double right_accel;
		public double left_feedforward;
		public double right_feedforward;
		public TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<Pose2dWithCurvature>(
				Pose2dWithCurvature.identity());
	}
}

