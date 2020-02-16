package frc.team3039.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3039.robot.AutoRoutineSelector;
import frc.team3039.robot.RobotMap;

/**
 * The Intake Class controls the front facing device used to collect "Power Cells"
 * from the floor and transfer them to the Indexer
 */
public class Intake extends Subsystem {
  private static Intake mInstance = new Intake();


  public VictorSPX intake;
  public Solenoid intakeTilt;

  public Intake() {
    intake = new VictorSPX(RobotMap.INTAKE_MOTOR_CAN_ID);
    intakeTilt = new Solenoid(RobotMap.INTAKE_TILT_PCM_ID);
    intake.setNeutralMode(NeutralMode.Brake);
  }

  public static Intake getInstance() {
    if (mInstance == null) {
      mInstance = new Intake();
    }
    return mInstance;
  }


  public void deploy() {
    intakeTilt.set(true);
  }

  public void retract() {
    intakeTilt.set(false);
  }

  public void setSpeed(double speed){
    intake.set(ControlMode.PercentOutput, speed);
  }

  @Override
  public void stop() {

  }

  @Override
  public boolean checkSystem() {
    return false;
  }

  @Override
  public void outputTelemetry(AutoRoutineSelector.DesiredMode mode) {

  }

}
