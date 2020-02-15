package frc.team3039.robot.commands;

import frc.team3039.robot.RobotContainer;
import frc.team3039.robot.subsystems.Hopper;

public class OverrideIndexing extends Command {

    private Hopper mHopper = RobotContainer.mHooper;
    private boolean overrideIndexing;

    public OverrideIndexing(boolean overrideIndexing){
        this.overrideIndexing = overrideIndexing;
    }

    @Override
    public void initialize() {
        mHopper.overrideIndexing(overrideIndexing);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
