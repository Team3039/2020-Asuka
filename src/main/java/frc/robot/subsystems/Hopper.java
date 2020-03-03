/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

/**
 * The Intake delivers "Power Cells" to this subsystem to be transfered to the
 * Shooter. This subsystem also "indexes" said "Power Cells" for rapid shooting
 */
public class Hopper extends SubsystemBase {

  public VictorSPX bouncer = new VictorSPX(RobotMap.bouncer);
  public TalonSRX feederBelts = new TalonSRX(RobotMap.feederBelts);
  public TalonSRX feederWheel = new TalonSRX(RobotMap.feederWheel);

  public DigitalInput topBeam = new DigitalInput(RobotMap.topBeam);
  public DigitalInput lowBeam = new DigitalInput(RobotMap.lowBeam);

  public enum HopperControlMode {
    IDLE,
    INTAKING,
    FEEDING,
    UNJAMMING
  }

  public HopperControlMode hopperControlMode = HopperControlMode.IDLE;

  public synchronized HopperControlMode getControlMode() {
    return hopperControlMode;
}

  public synchronized void setControlMode(HopperControlMode controlMode) {
    this.hopperControlMode = controlMode;
  }

  public boolean getTopBeam() {
    return topBeam.get();
  }

  public boolean getLowBeam() {
    return lowBeam.get();
  }

  public Hopper() {
    feederBelts.setInverted(true);
    feederWheel.setInverted(false);

    feederBelts.setNeutralMode(NeutralMode.Coast);
    feederWheel.setNeutralMode(NeutralMode.Coast);
  }

  public void runBelts(double percentOutput) {
    feederBelts.set(ControlMode.PercentOutput, percentOutput);
  }

  public void runBouncer(double percentOutput) {
    bouncer.set(ControlMode.PercentOutput, percentOutput);
  }

  public void runFeeder(double percentOutput) {
    feederWheel.set(ControlMode.PercentOutput, percentOutput);
  }

  public void runSystems(double percentA, double percentB, double percentC) {
    bouncer.set(ControlMode.PercentOutput, percentA);
    feederBelts.set(ControlMode.PercentOutput, percentB);
    feederWheel.set(ControlMode.PercentOutput, percentC);
  }

  public void stopSystems() {
    bouncer.set(ControlMode.PercentOutput, 0);
    feederBelts.set(ControlMode.PercentOutput, 0);
    feederWheel.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Top Beam", getTopBeam());
    SmartDashboard.putBoolean("Low Beam", getLowBeam());
    synchronized (Hopper.this) {
      switch (getControlMode()) {
        case IDLE:
          stopSystems();
          break;
        case INTAKING:
          if (getTopBeam() && getLowBeam()) {
            runSystems(.2, .5, .5);
          }
          else if (!getTopBeam() && getLowBeam()) {
            runBouncer(.2);
            runBelts(0);
            feederWheel.set(ControlMode.PercentOutput, .5);
          }
          else {
            runSystems(0, 0, 0);
          }
          break;
        case FEEDING:
          runSystems(.2, .5, .5);
          break;
        case UNJAMMING:
          runSystems(-.4, -.6, -.6);
          break;
      }
    }
  }
}
