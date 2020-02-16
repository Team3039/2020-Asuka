package frc.team3039.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3039.robot.AutoRoutineSelector.DesiredMode;
import frc.team3039.robot.RobotMap;
import frc.team3039.robot.loops.Loop;

import static frc.team3039.robot.Constants.*;

/**
 * The Shooter launches "Power Cells" from the robot to the "Power Port"
 */
public class Shooter extends Subsystem implements Loop {
    private static Shooter mInstance = new Shooter();

    public TalonFX shooterA,shooterB;

    public enum ShooterControlMode {
        OPEN_LOOP,
        CALCULATE,
        HOLD,
        SHOOT,
    }

    public ShooterControlMode mShooterControlMode = ShooterControlMode.OPEN_LOOP;

    @Override
    public void onStart(double timestamp) {
    }

    @Override
    public void onStop(double timestamp) {
    }

    @Override
    public void onLoop(double timestamp) {
        synchronized (Shooter.this) {
            switch (getControlMode()) {
                case OPEN_LOOP:
                    break;
                case CALCULATE:
                    System.out.println("Calculate Target RPM based on vision values and set speed");
                    break;
                case HOLD:
                    System.out.println("Once desired RPM is reached set KF value using formula and hold the RPM");
                    break;
                case SHOOT:
                    System.out.println("Use the Kf value calculated to shoot");
                    break;
                default:
                    System.out.println("Unknown hopper control mode");
                    break;
            }
        }
    }

    public synchronized ShooterControlMode getControlMode() {
        return mShooterControlMode;
    }

    public synchronized void setControlMode(ShooterControlMode controlMode) {
        this.mShooterControlMode = controlMode;
    }

    public Shooter() {

        shooterA = new TalonFX(RobotMap.SHOOTER_MOTOR_A_CAN_ID);
        shooterB = new TalonFX(RobotMap.SHOOTER_MOTOR_B_CAN_ID);

        shooterA.configFactoryDefault();
        shooterB.configFactoryDefault();
        shooterA.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

        shooterA.setInverted(false);
        shooterB.setInverted(true);

        shooterA.setNeutralMode(NeutralMode.Coast);
        shooterB.setNeutralMode(NeutralMode.Coast);

//        shooterA.configClosedLoopPeakOutput(kControlSlot, Constants.kShooterMaxPrecentOutput);

        shooterA.config_kP(0, kP_SHOOTER);
        shooterA.config_kI(0, kI_SHOOTER);
        shooterA.config_kD(0, kD_SHOOTER);
        shooterA.config_kF(0, kF_SHOOTER);
        shooterA.config_IntegralZone(0, kIZone_SHOOTER);

        shooterA.clearStickyFaults();
        shooterB.clearStickyFaults();

        shooterB.follow(shooterA);
    }

    public static Shooter getInstance() {
        if (mInstance == null) {
            mInstance = new Shooter();
        }
        return mInstance;
    }


    public void setShooterSpeed(double speed) {
        shooterA.set(ControlMode.PercentOutput, speed);
    }

    public void resetShooterPosition() {
        shooterA.setSelectedSensorPosition(0);
    }

    public double getShooterRotations() {
        return shooterA.getSelectedSensorPosition() / SHOOTER_OUTPUT_TO_ENCODER_RATIO / TICKS_PER_ROTATION;
    }

    public double getShooterRPM() {
        return shooterA.getSelectedSensorVelocity() / SHOOTER_OUTPUT_TO_ENCODER_RATIO / TICKS_PER_ROTATION * 10.0 * 60.0;
    }

    public void setShooterRPM(double rpm) {
        // double kF = (shooterA.getMotorOutputPercent() * 1023) / shooterA.getSelectedSensorVelocity();
        // shooterA.config_kF(0, kF);
        shooterA.set(ControlMode.Velocity, shooterRPMToNativeUnits(rpm));
    }

    public double shooterRPMToNativeUnits(double rpm) {
        return rpm * SHOOTER_OUTPUT_TO_ENCODER_RATIO * TICKS_PER_ROTATION / 10.0 / 60.0;
    }

    @Override
    public void zeroSensors() {
        shooterA.setSelectedSensorPosition(0,0,0);
        shooterB.setSelectedSensorPosition(0,0,0);
    }

    @Override
    public void stop() {

    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry(DesiredMode mode) {
        if(mode == DesiredMode.TEST){
            try{
            SmartDashboard.putNumber("Shooter RPM", getShooterRPM());
            SmartDashboard.putNumber("Shooter Output Percent", shooterA.getMotorOutputPercent());
            // SmartDashboard.putNumber("Shooter Velocity Native", shooterA.getSelectedSensorVelocity());
            // SmartDashboard.putNumber("Shooter Stator Current", shooterA.getStatorCurrent());
            // SmartDashboard.putNumber("Shooter Supply Current", shooterA.getSupplyCurrent());
        }catch(Exception e){
                System.out.println("Desired Mode Error in Shooter Subsystem");
            }
    }else if(mode == DesiredMode.COMPETITION){

        }
    }
}
