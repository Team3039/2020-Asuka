package frc.team3039.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.team3039.robot.commands.DriveSetSpeed;
import frc.team3039.robot.commands.OverrideIndexing;
import frc.team3039.robot.controller.GameController;
import frc.team3039.robot.controller.Playstation;
import frc.team3039.robot.subsystems.Drive;
import frc.team3039.robot.subsystems.Hopper;
import frc.team3039.robot.subsystems.Intake;
import frc.team3039.robot.subsystems.Shooter;
import frc.team3039.robot.subsystems.Turret;
import frc.team3039.robot.subsystems.Climber;

public class RobotContainer {

     private static RobotContainer mInstance;

     public static RobotContainer getInstance(){
         if (mInstance == null){
             mInstance = new RobotContainer();
         }
         return mInstance;
     }

    public final static Drive drive = Drive.getInstance();
    public final static Hopper hopper = Hopper.getInstance();
    public final static Intake intake =  Intake.getInstance();
    public final static Turret turret = Turret.getInstance();
    public final static Shooter shooter = Shooter.getInstance();
    public final static Climber climber =  Climber.getInstance();

    private GameController m_driver = new GameController(RobotMap.DRIVER_JOYSTICK_1_USB_ID, new Playstation());
    private GameController m_operator = new GameController(RobotMap.OPERATOR_JOYSTICK_1_USB_ID, new Playstation());

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    Button setSpeedOn = m_driver.getButtonA();
    setSpeedOn.whenPressed(new DriveSetSpeed(.5));

    Button setSpeedOff = m_driver.getButtonA();
    setSpeedOff.whenReleased(new DriveSetSpeed(0));

    Button overrideIndexing = m_driver.getStartButton();
    overrideIndexing.whenPressed(new OverrideIndexing(true));

  }

  public GameController getDriverController() {
    return m_driver;
  }

  public GameController getOperatorController() {
    return m_operator;
     }
 }
