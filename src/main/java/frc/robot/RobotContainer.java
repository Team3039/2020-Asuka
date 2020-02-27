package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.auto.Auto6BallSideInit;
import frc.robot.auto.Auto8BallCenterInit;
import frc.robot.commands.ActuateIntake;
import frc.robot.commands.Index;
import frc.robot.commands.RunIntake;
import frc.robot.commands.Shoot;
import frc.robot.commands.SpinWheel;
import frc.robot.controllers.PS4Gamepad;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ColorWheel;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class RobotContainer {
 
  public final static Drivetrain drivetrain = new Drivetrain();
  public final static Intake intake = new Intake();
  public final static Turret turret = new Turret();
  public final static Hopper hopper = new Hopper();
  public final static Shooter shooter = new Shooter();
  public final static Climber climber = new Climber();
  public final static ColorWheel colorWheel = new ColorWheel();

  public final static Auto6BallSideInit auto6BallSideInit = new Auto6BallSideInit();
  public final static Auto8BallCenterInit auto8BallCenterInit = new Auto8BallCenterInit();

  public static PS4Gamepad driverPad = new PS4Gamepad(RobotMap.DRIVER_JOYSTICK_1_USB_ID);
  public static PS4Gamepad operatorPad = new PS4Gamepad(RobotMap.OPERATOR_JOYSTICK_1_USB_ID);

  public static Timer timer = new Timer();

  //Declare Button Objects here
  //Driver Buttons
		Button driverTriangle = driverPad.getButtonTriangle();
		Button driverSquare = driverPad.getButtonSquare();
		Button driverCircle = driverPad.getButtonCircle();
    Button driverX = driverPad.getButtonX();
		Button driverShare = driverPad.getShareButton();
		Button driverOptions = driverPad.getOptionsButton();
		Button driverPadButton = driverPad.getButtonPad();
		Button driverL1 = driverPad.getL1();
		Button driverL2 = driverPad.getL2();
		Button driverL3 = driverPad.getL3();
		Button driverR1 = driverPad.getR1();
		Button driverR2 = driverPad.getR2();
		Button driverR3 = driverPad.getR3();

		//Operator Buttons
		Button operatorTriangle = operatorPad.getButtonTriangle();
		Button operatorSquare = operatorPad.getButtonSquare();
		Button operatorCircle = operatorPad.getButtonCircle();
		Button operatorX = operatorPad.getButtonX();
		Button operatorShare = operatorPad.getShareButton();
		Button operatorOptions = operatorPad.getOptionsButton();
		Button operatorPadButton = operatorPad.getButtonPad();
		Button operatorL1 = operatorPad.getL1();
		Button operatorL2 = operatorPad.getL2();
		Button operatorL3 = operatorPad.getL3();
		Button operatorR1 = operatorPad.getR1();
		Button operatorR2 = operatorPad.getR2();
		Button operatorR3 = operatorPad.getR3();
  
  public RobotContainer() {
    configureButtonBindings();
  }

  //Put Button Bindings Here
  private void configureButtonBindings() {

    //Driver
    driverShare.toggleWhenPressed(new Shoot(6450));
    driverX.toggleWhenPressed(new RunIntake());
    driverCircle.toggleWhenPressed(new ActuateIntake());
    driverTriangle.toggleWhenPressed(new Index());

    //Operator
	  // operatorCircle.whenPressed(new ActuateIntake());
    // operatorR2.whileHeld(new RunIntake());
    operatorTriangle.whenPressed(new SpinWheel(2));

    SmartDashboard.putData("Actuate Hood", new InstantCommand(()-> shooter.actuateHood()));
    SmartDashboard.putData("Lower Hood", new InstantCommand(()-> shooter.lowerHood()));
    SmartDashboard.putData("Set LEDs (On)", new InstantCommand(()-> turret.setLed(true)));
    SmartDashboard.putData("Set LEDs (Off)", new InstantCommand(()-> turret.setLed(false)));
    SmartDashboard.putData("Turret Manual", new InstantCommand(()-> turret.manualControl(RobotContainer.getOperator())));

    SmartDashboard.putData("Shooter 100%", new InstantCommand(()-> shooter.setShooterSpeed(1.0)));
    SmartDashboard.putData("Shooter 95%", new InstantCommand(()-> shooter.setShooterSpeed(.95)));
    SmartDashboard.putData("Shooter 90%", new InstantCommand(()-> shooter.setShooterSpeed(.9)));
    SmartDashboard.putData("Shooter 75%", new InstantCommand(()-> shooter.setShooterSpeed(.75)));
    SmartDashboard.putData("Shooter 50%", new InstantCommand(()-> shooter.setShooterSpeed(.5)));
    SmartDashboard.putData("Shooter 0%", new InstantCommand(()-> shooter.setShooterSpeed(0)));
    SmartDashboard.putData("Specific Shooter Speed", new InstantCommand(()-> shooter.setShooterSpeed(.78)));

  //Put climb on toggle
  }

  //Get Controller Objects
  public static PS4Gamepad getDriver() {
    return driverPad;
  }

  public static PS4Gamepad getOperator() {
    return operatorPad;
  }

  // public Command getAutonomousCommand() {
  //   // An ExampleCommand will run in autonomous
  //   // return autoCommand;
  // }
}
