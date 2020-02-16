package frc.team3039.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3039.robot.RobotMap;
import static frc.team3039.robot.Constants.*;

/**
 * The Shooter launches "Power Cells" from the robot to the "Power Port"
 */
public class Shooter extends SubsystemBase {

    public TalonFX shooterA = new TalonFX(RobotMap.SHOOTER_A);
    public TalonFX shooterB = new TalonFX(RobotMap.SHOOTER_B);

    public Shooter() {
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
    public void periodic() {
        SmartDashboard.putNumber("Shooter Rotations", getShooterRotations());
        SmartDashboard.putNumber("Shooter RPM", getShooterRPM());
        SmartDashboard.putNumber("Shooter Output Percent", shooterA.getMotorOutputPercent());
        // SmartDashboard.putNumber("Shooter Velocity Native", shooterA.getSelectedSensorVelocity());
        // SmartDashboard.putNumber("Shooter Stator Current", shooterA.getStatorCurrent());
        // SmartDashboard.putNumber("Shooter Supply Current", shooterA.getSupplyCurrent());
     }
}
