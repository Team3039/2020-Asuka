/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auto.Auto6BallSideInit;
import frc.robot.auto.Auto8BallCenterInit;
import frc.robot.auto.AutoSafe;
import frc.robot.auto.AutoTestA;
import frc.robot.auto.AutoTestB;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
  private Command autoCommand;
  public Auto6BallSideInit auto6BallInitSide = new Auto6BallSideInit();
  public Auto8BallCenterInit auto8BallCenterInit = new Auto8BallCenterInit();
  public AutoSafe autoSafe = new AutoSafe();
  public AutoTestA autoTestA = new AutoTestA();
  public AutoTestB autoTestB = new AutoTestB();

  private RobotContainer robotContainer;

   //Vision Information
   public static double targetValid; //Whether the limelight has any valid targets (0 or 1)
   public static double targetX; //Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
   public static double targetY; //Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
   public static double targetArea; //Target Area (0% of image to 100% of image)
   
   SendableChooser<Command> autoChooser = new SendableChooser<>();

   UsbCamera usbCamera = new UsbCamera("DriveCam", 0);

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
    SmartDashboard.putNumber("Target AREA", RobotContainer.turret.getTargetArea());
    SmartDashboard.putData("Auto mode", autoChooser);
    autoChooser.addOption("Auto 6 Ball Side Init", auto6BallInitSide);
    autoChooser.addOption("Auto 8 Ball Center Init", auto8BallCenterInit);
    autoChooser.addOption("Auto Safe", autoSafe);
    autoChooser.addOption("Auto TestA", autoTestA);
    autoChooser.addOption("Auto TestB", autoTestB);
  }

  @Override
  public void robotPeriodic() {
    //Gather Vision Info=\
    targetValid = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    targetX = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    targetY = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    targetArea = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    RobotContainer.drivetrain.resetEncoders();
  }

  @Override
  public void autonomousInit() {
    autoCommand = autoChooser.getSelected();

    // schedule the autonomous command (example)
    if (autoChooser != null) {
      autoCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (autoCommand != null) {
      autoCommand.cancel();
    }
    usbCamera.setVideoMode(VideoMode.PixelFormat.kYUYV, 320, 180, 60);
    
  }

  @Override
  public void teleopPeriodic() {
    RobotContainer.drivetrain.joystickControl(RobotContainer.getDriver());
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }
}
