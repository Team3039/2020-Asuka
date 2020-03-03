/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.RobotMap;
import static frc.robot.Constants.*;

/**
 * The Shooter launches "Power Cells" from the robot to the "Power Port"
 */
public class Shooter extends SubsystemBase {

    public TalonFX shooterA = new TalonFX(RobotMap.shooterA);
    public TalonFX shooterB = new TalonFX(RobotMap.shooterB);

    public Solenoid hood = new Solenoid(RobotMap.hood);

    private enum ShooterControlMode {
        IDLE,
        HOLD,
        CALCULATE,
        SHOOT,
    }

    public ShooterControlMode shooterControlMode = ShooterControlMode.IDLE;

    public synchronized ShooterControlMode getControlMode() {
        return shooterControlMode;
    }

    public synchronized void setControlMode(ShooterControlMode controlMode) {
        this.shooterControlMode = controlMode;
      }

    public void actuateHood() {
        hood.set(true);
    }

    public void lowerHood() {
        hood.set(false);
    }

    public Shooter() {
        shooterA.configFactoryDefault();
        shooterB.configFactoryDefault();
        shooterA.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);


        shooterA.setInverted(true);
        shooterA.setSensorPhase(true);

        shooterB.setInverted(false);
        shooterB.setSensorPhase(true);

        shooterA.setNeutralMode(NeutralMode.Coast);
        shooterB.setNeutralMode(NeutralMode.Coast);

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

    public double calculateDesiredOutput(double x) {
        double a = 36;
        double b = -485.8;
        double c = 6436;
        double rpmGenerated = (a * (Math.pow(x,2))) + (b * x) + c; 
        MathUtil.clamp(rpmGenerated, 4000, 8000);
        return rpmGenerated;
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
    }
}
