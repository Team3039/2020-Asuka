package frc.team3039.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team3039.robot.RobotContainer;

public class DriveSetSpeed extends CommandBase {
    double speed;

    public DriveSetSpeed(double speed){
        this.speed = speed;
    }

    @Override
    public void initialize() {
        RobotContainer.drive.setSpeed(speed);
    }

    @Override
    public void execute() {
        System.out.println("Drive");
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {}

}
