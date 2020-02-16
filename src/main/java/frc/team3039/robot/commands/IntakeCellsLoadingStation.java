package frc.team3039.robot.commands;

import static frc.team3039.robot.RobotContainer.hopper;
import frc.team3039.robot.subsystems.Hopper;

public class IntakeCellsLoadingStation extends ExtraTimeoutCommand {

    private boolean isBallsLoaded = false;
    private int ballsLoaded = 0;

    public IntakeCellsLoadingStation(){}

    @Override
    public void initialize() {
        resetExtraOneTimer();
        hopper.setHopperSpeed(.5);
        hopper.setFeederOmniSpeed(.5);
        hopper.setFeederBeltsSpeed(.75);
        hopper.setFeederBeltsSpeed(.5);
    }

    @Override
    public void execute() {
        while (ballsLoaded != 2) {
            if (hopper.getHopperCurrent() < hopper.HOPPER_CURRENT_INDEX_THRESHOLD) {
                hopper.setControlMode(hopper.mHopperControlMode.LOADING);
                if (hopper.getFeederOmniCurrent() > hopper.HOPPER_CURRENT_INDEX_THRESHOLD) {
                    ballsLoaded++;
                    startExtraOneTimeout(.5);
                }
                resetExtraOneTimer();
            }else{
                hopper.setControlMode(hopper.mHopperControlMode.UNJAMMING);
            }
            isBallsLoaded = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isBallsLoaded;
    }

    @Override
    public void end() {hopper.stopHopper();}

    @Override
    public void interrupted(){end();}
}
