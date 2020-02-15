package frc.team3039.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3039.robot.AutoRoutineSelector.DesiredMode;
import frc.team3039.robot.RobotMap;
import frc.team3039.robot.loops.Loop;

public class Hopper extends Subsystem implements Loop {

    private static Hopper mInstance = new Hopper();

    public TalonSRX revolverMotor,feederOmni;
    public VictorSPX kickerOmni, feederBelts;


    public double kOmniCurrentSpikeThreshold;
    public double kRevolverSpikeCurrentThreshold;

    private boolean overrideIndexing;

    public enum HopperControlMode {
        OPEN_LOOP,
        INDEXING,
        SHOOTING,
        UNJAMMING,
    }

    public HopperControlMode mHopperControlMode = HopperControlMode.OPEN_LOOP;

        @Override
        public void onStart(double timestamp) {
            overrideIndexing = false;
        }

        @Override
        public void onStop(double timestamp) {
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Hopper.this) {
                switch (getControlMode()) {
                    case INDEXING:
                        setHooperIndexing(.5);
                        isIndexOverrode();
                        break;
                    case SHOOTING:
                        setHooperShooting(.75);
                        break;
                    case UNJAMMING:
                        System.out.println("Hooper jammed starting UNJAM sequence");
                        setHooperJamming();
                        break;
                    case OPEN_LOOP:
                        break;
                    default:
                        System.out.println("Unknown hopper control mode");
                        break;
                }
            }
        }

    public synchronized HopperControlMode getControlMode() {
        return mHopperControlMode;
    }

    public synchronized void setControlMode(HopperControlMode controlMode) {
        this.mHopperControlMode = controlMode;
    }

    public Hopper(){
        revolverMotor = new TalonSRX(RobotMap.REVOLVER_SPIN_MOTOR_CAN_ID);
        feederOmni = new TalonSRX(RobotMap.REVOLVER_FEEDER_OMNI_CAN_ID);

        kickerOmni = new VictorSPX(RobotMap.REVOLVER_KICKER_OMNI_CAN_ID);
        feederBelts = new VictorSPX(RobotMap.REVOLVER_FEEDER_BELTS_CAN_ID);

        setBrakeMode(true);

    }

    public static Hopper getInstance() {
        if (mInstance == null) {
            mInstance = new Hopper();
        }
        return mInstance;
    }

    public void overrideIndexing(boolean override){
        this.overrideIndexing = override;
    }

    public boolean isIndexOverrode(){
            return overrideIndexing;
    }


    public void setRevolverSpeed(double speed){
        if(speed != 0.0){
            revolverMotor.set(ControlMode.PercentOutput, speed);
        }else{
            revolverMotor.set(ControlMode.PercentOutput, 0);
        }
    }

    public void setFeederOmniSpeed(double speed){
        feederOmni.set(ControlMode.PercentOutput, speed);
    }

    public void setKickerOmniSpeed(double speed){
        kickerOmni.set(ControlMode.PercentOutput, speed);
    }

    public void setFeederBeltsSpeed(double speed){
        feederBelts.set(ControlMode.PercentOutput, speed);
    }

    public void setHooperIndexing(double speed){
        setRevolverSpeed(speed);
        setFeederOmniSpeed(speed);
        setKickerOmniSpeed(speed * 1.5);
        setFeederBeltsSpeed(speed);
    }

    public void setHooperShooting(double speed){
        setRevolverSpeed(speed);
        setFeederOmniSpeed(speed);
        setKickerOmniSpeed(speed);
        setFeederBeltsSpeed(speed);
    }

    public void setHooperJamming(){
        setRevolverSpeed(-.6);
        setKickerOmniSpeed(.75);
    }

    public void stopHopper(){
        setRevolverSpeed(0.0);
        setFeederOmniSpeed(0.0);
        setKickerOmniSpeed(0.0);
        setFeederBeltsSpeed(0.0);
    }

    public double getRevolverCurrent(){ return revolverMotor.getStatorCurrent(); }

    public double getFeederOmniCurrent(){  return feederOmni.getStatorCurrent(); }

    public void setBrakeMode(boolean setBrake){
        if(setBrake){
            revolverMotor.setNeutralMode(NeutralMode.Brake);
            feederOmni.setNeutralMode(NeutralMode.Brake);

            kickerOmni.setNeutralMode(NeutralMode.Brake);
            feederBelts.setNeutralMode(NeutralMode.Brake);

            System.out.println("Revolver Motors in Brake");
        }else{
            revolverMotor.setNeutralMode(NeutralMode.Coast);
            feederOmni.setNeutralMode(NeutralMode.Coast);

            kickerOmni.setNeutralMode(NeutralMode.Coast);
            feederBelts.setNeutralMode(NeutralMode.Coast);

            System.out.println("Revolver Motors in Coast");
        }
    }

    @Override
    public void stop() {

    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry(DesiredMode mode) {
        if (mode == DesiredMode.TEST) {
            try {
                SmartDashboard.putNumber("Revolver Motor Current: " , getRevolverCurrent());
                SmartDashboard.putNumber("Feeder Omni Current: ", getFeederOmniCurrent());
            } catch (Exception e) {
                System.out.println("Desired Mode Error");
            }
        } else if (mode == DesiredMode.COMPETITION) {

        }
    }
}
