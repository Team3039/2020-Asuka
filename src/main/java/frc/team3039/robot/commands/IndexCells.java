package frc.team3039.robot.commands;

import frc.team3039.robot.RobotContainer;
import frc.team3039.robot.subsystems.Hopper;

public class IndexCells extends ExtraTimeoutCommand {

    private Hopper mHopper = RobotContainer.mHooper;
    private boolean isBallsLoaded;
    private int ballsLoaded;
    private boolean isOverrode = mHopper.isIndexOverrode();

    public IndexCells(){}

    @Override
    public void initialize() {
        isBallsLoaded = false;
        ballsLoaded = 0;
        resetExtraOneTimer();
        resetExtraTwoTimer();
    }

    @Override
    public void execute() {
        while (ballsLoaded != 2) {
            if (mHopper.getRevolverCurrent() < mHopper.kRevolverSpikeCurrentThreshold) {
                mHopper.setControlMode(Hopper.HopperControlMode.INDEXING);
                if (mHopper.getFeederOmniCurrent() > mHopper.kOmniCurrentSpikeThreshold) {
                    ballsLoaded++;
                    startExtraOneTimeout(.5);
                }
                resetExtraOneTimer();
            }else{
                startExtraTwoTimeout(.75);
                mHopper.setControlMode(Hopper.HopperControlMode.UNJAMMING);

            }
            resetExtraTwoTimer();
        }
        isBallsLoaded = true;
    }

    @Override
    public boolean isFinished() {
        return isBallsLoaded || isOverrode;
    }

    @Override
    public void end() {
        mHopper.setControlMode(Hopper.HopperControlMode.OPEN_LOOP);
        mHopper.stopHopper();
        ballsLoaded = 0;

    }

    @Override
    public void interrupted(){end();}
}
