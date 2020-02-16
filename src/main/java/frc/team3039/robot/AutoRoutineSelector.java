package frc.team3039.robot;

import java.util.Optional;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3039.robot.auto.AutoRoutineBase;
import frc.team3039.robot.auto.actions.CharacterizeDrivebase;
import frc.team3039.robot.auto.routines.*;

/* AutoRoutineSelector is where the auto selection and processing takes place. Available options are a Starting position a
    DesiredMode which is used to change robot action at the start of autonomous of teleOp and the AutoRoutine that you
    would like to run.
 */

public class AutoRoutineSelector {

    public enum StartingPosition {
        INITIATION_LINE
    }

    public enum DesiredMode {
        TEST, PRACTICE, COMPETITION
    }

    enum AutoChooser {
        DO_NOTHING, TRENCH, STEAL, RENDEZVOUS, SAFE, TEST, TEST_CONTROL_FLOW, DRIVE_CHARACTERIZATION_STRAIGHT,
        DRIVE_CHARACTERIZATION_TURN
    }

    private DesiredMode mCachedDesiredMode = null;
    private StartingPosition mCachedStartingPosition = null;
    private AutoChooser mCachedAutoSelected = null;

    private SendableChooser<DesiredMode> mModeChooser;
    private SendableChooser<StartingPosition> mStartPositionChooser;
    private SendableChooser<AutoChooser> mAutoChooser;

    private Optional<AutoRoutineBase> mAutoMode = Optional.empty();

    public AutoRoutineSelector() {
        mStartPositionChooser = new SendableChooser<StartingPosition>();
        mStartPositionChooser.setDefaultOption("Initiation Line", StartingPosition.INITIATION_LINE);

        SmartDashboard.putData("Starting Position", mStartPositionChooser);

        mModeChooser = new SendableChooser<DesiredMode>();
        mModeChooser.setDefaultOption("Competition Mode", DesiredMode.COMPETITION);
        mModeChooser.addOption("Test Mode", DesiredMode.TEST);
        mModeChooser.addOption("Practice Mode", DesiredMode.PRACTICE);

        SmartDashboard.putData("Desired mode", mModeChooser);

        mAutoChooser = new SendableChooser<AutoChooser>();

        mAutoChooser.setDefaultOption("Do Nothing", AutoChooser.DO_NOTHING);

        mAutoChooser.addOption("Trench 8 Ball Auto", AutoChooser.TRENCH);
        mAutoChooser.addOption("Trench Steal Ball Auto", AutoChooser.STEAL);

        mAutoChooser.addOption("Rendezvous 5 Ball Auto", AutoChooser.RENDEZVOUS);
        mAutoChooser.addOption("Safe Ball Auto", AutoChooser.SAFE);

        mAutoChooser.addOption("Test", AutoChooser.TEST);
        mAutoChooser.addOption("Test control flow", AutoChooser.TEST_CONTROL_FLOW);
        mAutoChooser.addOption("Drive Characterization Straight", AutoChooser.DRIVE_CHARACTERIZATION_STRAIGHT);
        mAutoChooser.addOption("Drive Characterization Turn", AutoChooser.DRIVE_CHARACTERIZATION_TURN);

        SmartDashboard.putData("Autonomous", mAutoChooser);
    }

    public void updateModeCreator() {
        DesiredMode desiredMode = mModeChooser.getSelected();
        StartingPosition startingPosition = mStartPositionChooser.getSelected();
        AutoChooser autoSelected = mAutoChooser.getSelected();
        if (mCachedDesiredMode != desiredMode || startingPosition != mCachedStartingPosition
                || autoSelected != mCachedAutoSelected) {
            System.out.println("Auto selection changed, updating creator: desiredMode-> " + desiredMode.name()
                    + "selected auto -> " + autoSelected.name() + ", starting position-> " + startingPosition.name());
            mAutoMode = getAutoModeForParams(desiredMode, autoSelected, startingPosition);
        }
        mCachedDesiredMode = desiredMode;
        mCachedStartingPosition = startingPosition;
        mCachedAutoSelected = autoSelected;
    }

    private Optional<AutoRoutineBase> getAutoModeForParams(DesiredMode mode, AutoChooser auto,
            StartingPosition position) {
        switch (auto) {
        case DO_NOTHING:
            return Optional.of(new AutoDoNothing());
        case TRENCH:
            return Optional.of(new AutoTrench8Ball());
        // case RENDEZVOUS:
        //     return Optional.of(new AutoRendezvous5Ball());
        case SAFE:
            return Optional.of(new AutoSafe());
        case STEAL:
            return Optional.of(new AutoTrenchSteal());
        case TEST:
            return Optional.of(new AutoTest());
        case TEST_CONTROL_FLOW:
            return Optional.of(new AutoTestControlFlow());
        case DRIVE_CHARACTERIZATION_STRAIGHT:
            return Optional.of(new CharacterizeDrivebase(false, false));
        case DRIVE_CHARACTERIZATION_TURN:
            return Optional.of(new CharacterizeDrivebase(false, true));
        default:
            break;
        }

        System.err.println("No valid auto mode found for  " + auto);
        return Optional.empty();
    }

    public void reset() {
        mAutoMode = Optional.empty();
        mCachedDesiredMode = null;
    }

    public void outputTelemetry() {
        SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
        SmartDashboard.putString("StartingPositionSelected", mCachedStartingPosition.name());
        SmartDashboard.putString("AutoSelected", mCachedAutoSelected.name());
    }

    public Optional<AutoRoutineBase> getAutoMode() {
        return mAutoMode;
    }

    public DesiredMode getDesiredMode() {
        return mModeChooser.getSelected();
    }

}
