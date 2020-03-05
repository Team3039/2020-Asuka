package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.Climb;
import frc.robot.commands.DeployClimb;
import frc.robot.commands.DeployClimbArms;
import frc.robot.commands.RetractClimbArms;
import frc.robot.commands.SetHopperIdleMode;
import frc.robot.commands.SetHopperUnjamMode;
import frc.robot.commands.sequences.FeedCells;
import frc.robot.commands.sequences.IndexCells;
import frc.robot.commands.sequences.IntakeCells;
import frc.robot.commands.sequences.ResetHopper;
import frc.robot.commands.sequences.ResetShooter;
import frc.robot.commands.sequences.ShootFarShot;
import frc.robot.commands.sequences.ShootMidShot;
import frc.robot.commands.sequences.ShootNearShot;
import frc.robot.controllers.PS4Gamepad;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ColorWheel;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
public class RobotContainer {
 
  public final static Drive drive = Drive.getInstance();
  public final static Intake intake = new Intake();
  public final static Turret turret = new Turret();
  public final static Hopper hopper = new Hopper();
  public final static Shooter shooter = new Shooter();
  public final static Climber climber = new Climber();
  public final static ColorWheel colorWheel = new ColorWheel();

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
		// Button driverL2 = driverPad.getL2();
		Button driverL3 = driverPad.getL3();
		Button driverR1 = driverPad.getR1();
		// Button driverR2 = driverPad.getR2();
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
		Button operatorR1 = operatorPad.getR1();
		Button operatorR2 = operatorPad.getR2();

  
  public RobotContainer() {
    configureButtonBindings();
  }

  //Put Button Bindings Here
  private void configureButtonBindings() {

    //Driver
    driverL1.whileHeld(new DeployClimb());
    driverR1.whileHeld(new Climb());
    driverShare.whileHeld(new DeployClimbArms());
    driverOptions.whileHeld(new RetractClimbArms());


    //Operator
    //When X is pressed it turns on the shooter to a set RPM (6350) raises the hood and starts tracking
    operatorX.whenPressed(new ShootFarShot());
    //When Circle is pressed it turns on the shooter to a set RPM (5250) raises the hood and starts tracking
    operatorCircle.whenPressed(new ShootMidShot());
    //When Triangle is pressed it turns on the shooter to a set RPM(4800) raises the hood and starts tracking
    operatorTriangle.whenPressed(new ShootNearShot());
    //When Right Bumper is held the intake comes down and the intaking sequence runs 
    operatorR1.whileHeld(new IntakeCells());
    operatorR1.whenReleased(new IndexCells());
    //When Right Trigger is held the feeding sequence runs 
    operatorR2.whileHeld(new FeedCells());
    // When the feed command ends, the systems are all reset
    operatorR2.whenReleased(new ResetHopper());
    operatorR2.whenReleased(new ResetShooter());

    operatorL1.whileHeld(new SetHopperUnjamMode());
    operatorL1.whenReleased(new SetHopperIdleMode());
    
  }

  //Get Controller Objects
  public static PS4Gamepad getDriver() {
    return driverPad;
  }

  public static PS4Gamepad getOperator() {
    return operatorPad;
  }
}
