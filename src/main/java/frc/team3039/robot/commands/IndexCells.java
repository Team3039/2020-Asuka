package frc.team3039.robot.commands;

import static frc.team3039.robot.RobotContainer.hopper;

import frc.team3039.robot.subsystems.Hopper.HopperControlMode;

public class IndexCells extends ExtraTimeoutCommand {

    private boolean isBallsLoaded;
    private int ballsLoaded;
    private boolean isOverrode = hopper.isOverrode;

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
            if (hopper.getFeederOmniCurrent() < hopper.HOPPER_CURRENT_INDEX_THRESHOLD) {
                hopper.setControlMode(HopperControlMode.LOADING);
                if (hopper.getFeederOmniCurrent() > hopper.HOPPER_CURRENT_INDEX_THRESHOLD) {
                    ballsLoaded++;
                    startExtraOneTimeout(.5);
                }
                resetExtraOneTimer();
            }
            else{
                startExtraTwoTimeout(.75);
                hopper.setControlMode(HopperControlMode.UNJAMMING);

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
        hopper.setControlMode(HopperControlMode.OPEN_LOOP);
        hopper.stopHopper();
        ballsLoaded = 0;

    }

    @Override
    public void interrupted(){end();}
}
