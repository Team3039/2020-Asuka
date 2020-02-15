package frc.team3039.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public abstract class Command extends CommandBase {

    public Command(){}

    @Override
    public abstract void initialize();

    @Override
    public abstract void execute();

    @Override
    public abstract boolean isFinished();

    @Override
    public abstract void end(boolean interrupted);

}
