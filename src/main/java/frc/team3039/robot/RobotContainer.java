 /*----------------------------------------------------------------------------*/
 /* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
 /* Open Source Software - may be modified and shared by FRC teams. The code   */
 /* must be accompanied by the FIRST BSD license file in the root directory of */
 /* the project.                                                               */
 /*----------------------------------------------------------------------------*/

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

 /**
  * This class is where the bulk of the robot should be declared.  Since Command-based is a
  * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
  * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
  * (including subsystems, commands, and button mappings) should be declared here.
  */
 public class RobotContainer {
   // The robot's subsystems and commands are defined here...

     private static RobotContainer mInstance;

     public static RobotContainer getInstance(){
         if (mInstance == null){
             mInstance = new RobotContainer();
         }
         return mInstance;
     }

     private GameController m_driver = new GameController(RobotMap.DRIVER_JOYSTICK_1_USB_ID, new Playstation());
     private GameController m_operator = new GameController(RobotMap.OPERATOR_JOYSTICK_1_USB_ID, new Playstation());

     public final static Drive mDrive = Drive.getInstance();
     public final static Hopper mHooper = Hopper.getInstance();

   /**
    * The container for the robot.  Contains subsystems, OI devices, and commands.
    */
   public RobotContainer() {
     // Configure the button bindings
     configureButtonBindings();
   }

   /**
    * Use this method to define your button->command mappings.  Buttons can be created by
    * instantiating a {@link GenericHID} or one of its subclasses ({@link
    * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
    * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
    */
   private void configureButtonBindings() {

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
