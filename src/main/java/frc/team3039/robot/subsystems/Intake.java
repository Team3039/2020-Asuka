package frc.team3039.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3039.robot.RobotMap;

/**
 * The Intake Class controls the front facing device used to collect "Power Cells"
 * from the floor and transfer them to the Indexer
 */
public class Intake extends SubsystemBase {

  public VictorSPX intake = new VictorSPX(RobotMap.INTAKE);
  public Solenoid intakeTilt = new Solenoid(RobotMap.INTAKE_TILT);

  public Intake() {
    intake.setNeutralMode(NeutralMode.Brake);
  }

  public void deploy() {
    intakeTilt.set(true);
  }

  public void retract() {
    intakeTilt.set(false);
  }

  public void start() {
    intake.set(ControlMode.PercentOutput, .65);
  }

  public void stop() {
    intake.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
  }
}
