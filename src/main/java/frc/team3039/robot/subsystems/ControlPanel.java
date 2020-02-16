/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team3039.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team3039.robot.AutoRoutineSelector;
import frc.team3039.robot.RobotContainer;
import frc.team3039.robot.RobotMap;
import frc.team3039.robot.controller.GameController;
import frc.team3039.robot.loops.ILooper;
import frc.team3039.robot.loops.Loop;

public class ControlPanel extends Subsystem {
  private static ControlPanel mInstance = new ControlPanel();

  public Solenoid deployer;
  public TalonSRX spinner;

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch colorMatcher = new ColorMatch();

  public static final Color kBlueTarget = ColorMatch.makeColor(0.136, 0.412, 0.450);
  public static final Color kGreenTarget = ColorMatch.makeColor(0.196, 0.557, 0.246);
  public static final Color kRedTarget = ColorMatch.makeColor(0.475, 0.371, 0.153);
  public static final Color kYellowTarget = ColorMatch.makeColor(0.293, 0.561, 0.144);
  public static final double OPERATOR_ROT = .65;
  private static final int COLOR_WHEEL_PPR = 0;

  public enum ControlPanelControlMode{
    OPEN_LOOP,
    JOYSTICK,
    POSITION,
    ROTATION,
  }

  public ControlPanelControlMode mControlPanelControlMode = ControlPanelControlMode.OPEN_LOOP;

  private Loop mLoop = new Loop() {

    @Override
    public void onStart(double timestamp) {

    }

    @Override
    public void onStop(double timestamp) {
    }

    @Override
    public void onLoop(double timestamp) {
      synchronized (ControlPanel.this) {
        switch (getControlMode()) {
          case OPEN_LOOP:
            break;
          case JOYSTICK:
            manualControl(RobotContainer.getInstance().getOperatorController());
            break;
          case POSITION:
            getColor();
            break;
          case ROTATION:
            System.out.println("Rotation control");
            break;
          default:
            break;
        }
      }
    }
  };

  @Override
  public void registerEnabledLoops(ILooper in) {
    in.register(mLoop);
  }

  public synchronized ControlPanelControlMode getControlMode() {
    return mControlPanelControlMode;
  }

  public synchronized void setControlMode(ControlPanelControlMode controlMode) {
    this.mControlPanelControlMode = controlMode;
  }

  public ControlPanel() {
    deployer = new Solenoid(RobotMap.CONTROL_PANEL_DEPLOY_PCM_ID);
    spinner = new TalonSRX(RobotMap.CONTROL_PANEL_MOTOR_CAN_ID);

    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kGreenTarget);
    colorMatcher.addColorMatch(kRedTarget);
    colorMatcher.addColorMatch(kYellowTarget);
    colorMatcher.setConfidenceThreshold(0.80);
  }

  public static ControlPanel getInstance() {
    if (mInstance == null) {
      mInstance = new ControlPanel();
    }
    return mInstance;
  }

  private ColorMatchResult matchedResult = new ColorMatchResult(Color.kBlack, 0);

  // Rev Color threshold
  // blue 0.143, 0.427, 0.429
  // green 0.197, 0.561, 0.240
  // red 0.561, 0.232, 0.114
  // yellow 0.361, 0.524, 0.113

  public Color getColor() {

    Color detectedColor = colorSensor.getColor();
    String colorString;
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
    return match.color;
  }

  public void manualControl(GameController gp) {
    if(deployer.get()) {
      double rot = -1 * gp.getRightXAxis() * OPERATOR_ROT;
      double output = rot;
      spinner.set(ControlMode.PercentOutput, output);
    }
    else {
      spinner.set(ControlMode.PercentOutput, 0);
    }
  }

  public void spin(int rotations) {
    spinner.set(ControlMode.Position, COLOR_WHEEL_PPR * rotations * 6.25);
  }

  @Override
  public void zeroSensors() {
    spinner.setSelectedSensorPosition(0,0,0);
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
