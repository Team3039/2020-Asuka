package frc.robot;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.Climb;
import frc.robot.commands.DeployClimb;
import frc.robot.commands.DeployClimbArms;
import frc.robot.commands.RetractClimbArms;
import frc.robot.controllers.PS4Gamepad;
import frc.robot.statemachines.Cycler;
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

  public final static Cycler cycler = new Cycler();


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
    Button operatorDpadUp = operatorPad.getDPadUp();
    Button operatorDpadLeft = operatorPad.getDPadLeft();
    Button operatorDpadRight = operatorPad.getDPadRight();
    Button operatorDpadDown = operatorPad.getDPadDown();
  
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
    //When Circle is pressed it turns on the shooter to a set RPM (5250) raises the hood and starts tracking
    //When Triangles is pressed it turns on the shooter to a set RPM(4800) raises the hood and starts tracking
    
    //When Right Bumper is held the intake comes down and the intaking sequence runs 
    //When Right Bumper is relased the intake comes up and the intaking sequence runs

    //When Right Trigger is held the feeding sequence runs 
    //When Right Trigger is relased the shooter stop, the feeding sequence stops, the intake stops, a waitcommand is started for .25 
    //second and then the hood comes down


    SmartDashboard.putNumber("RPM", shooter.getShooterRPM());
    SmartDashboard.putNumber("Gyro", drive.getGyroFusedHeadingAngleDeg());
    
    // SmartDashboard.putNumber("Target AREA", turret.getTargetArea());
    // SmartDashboard.putNumber("getRPM", RobotContainer.shooter.getShooterRPM());
    // SmartDashboard.putNumber("Percent Output", RobotContainer.shooter.shooterA.getMotorOutputVoltage());

    SmartDashboard.putData("ResetPose", new InstantCommand(()-> drive.resetGyroYawAngle(0)));
    // SmartDashboard.putData("Actuate Climb", new InstantCommand(()-> climber.actuateClimb()));
    // SmartDashboard.putData("Lower Hood", new InstantCommand(()-> shooter.lowerHood()));
    // SmartDashboard.putData("Reset Turret Pose", new InstantCommand(()-> turret.resetTurretPosition()));
    // SmartDashboard.putData("Turret Manual", new InstantCommand(()-> turret.manualControl(RobotContainer.getOperator())));
    // SmartDashboard.putData("LEDs On", new InstantCommand(()-> turret.setLed(true)));
    // SmartDashboard.putData("LEDs Off", new InstantCommand(()-> turret.setLed(false)));
    // SmartDashboard.putData("TrackMode", new InstantCommand(()-> turret.setCamMode(true)));
    // SmartDashboard.putData("DriveMode", new InstantCommand(()-> turret.setCamMode(false)));

    // SmartDashboard.putData("6000", new InstantCommand(()-> shooter.setShooterRPM(6000)));
    // SmartDashboard.putData("5000", new InstantCommand(()-> shooter.setShooterRPM(5000)));
    // SmartDashboard.putData("Shooter 85%", new InstantCommand(()-> shooter.setShooterSpeed(.9)));
    // SmartDashboard.putData("Shooter 80%", new InstantCommand(()-> shooter.setShooterSpeed(.8)));
    // SmartDashboard.putData("Shooter 75%", new InstantCommand(()-> shooter.setShooterSpeed(.75)));
    // SmartDashboard.putData("Shooter 70%", new InstantCommand(()-> shooter.setShooterSpeed(.7)));
    // SmartDashboard.putData("Shooter 65%", new InstantCommand(()-> shooter.setShooterSpeed(.65))); 
    // SmartDashboard.putData("Shooter 50%", new InstantCommand(()-> shooter.setShooterSpeed(.5)));
    // SmartDashboard.putData("Shooter 0%", new InstantCommand(()-> shooter.setShooterSpeed(0)));
    // SmartDashboard.putData("Specific Shooter Speed", new InstantCommand(()-> shooter.setShooterSpeed(.78)));
  }

  //Get Controller Objects
  public static PS4Gamepad getDriver() {
    return driverPad;
  }

  public static PS4Gamepad getOperator() {
    return operatorPad;
  }
}
