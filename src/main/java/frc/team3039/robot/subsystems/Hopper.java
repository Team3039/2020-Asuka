package frc.team3039.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team3039.robot.AutoRoutineSelector.DesiredMode;
import frc.team3039.robot.RobotMap;
import frc.team3039.robot.loops.ILooper;
import frc.team3039.robot.loops.Loop;

public class Hopper extends Subsystem {

    public TalonSRX HopperMotor,feederOmni;
    public VictorSPX kickerOmni, feederBelts;

    public boolean isLoading;
    public boolean isJammed;
    public boolean isOverrode = false;

    public final double HOPPER_CURRENT_INDEX_THRESHOLD = 0;

    public static enum HopperControlMode {
        OPEN_LOOP,
        LOADING,
        SHOOTING,
        UNJAMMING,
    }

    public HopperControlMode mHopperControlMode = HopperControlMode.OPEN_LOOP;

    private final Loop mLoop = new Loop() {

        @Override
        public void onStart(double timestamp) {
            isLoading = false;
            isJammed = false;
        }

        @Override
        public void onStop(double timestamp) {
        }

        @Override
        public void onLoop(double timestamp) {
            synchronized (Hopper.this) {
                switch (getControlMode()) {
                    case LOADING:
                        setHopperSpeed(.65);
                        break;
                    case SHOOTING:
                        break;
                    case UNJAMMING:
                        setHopperSpeed(-.5);
                        break;
                    case OPEN_LOOP:
                        break;
                    default:
                        System.out.println("Unknown Hopper control mode");
                        break;
                }
            }
        }
    };

    @Override
    public void registerEnabledLoops(ILooper in) {
        in.register(mLoop);
    }

    public synchronized HopperControlMode getControlMode() {
        return mHopperControlMode;
    }

    public synchronized void setControlMode(HopperControlMode controlMode) {
        this.mHopperControlMode = controlMode;
    }

    public Hopper(){
        HopperMotor = new TalonSRX(RobotMap.HOPPER_SPIN_MOTOR_CAN_ID);
        feederOmni = new TalonSRX(RobotMap.HOPPER_FEEDER_OMNI_CAN_ID);

        kickerOmni = new VictorSPX(RobotMap.HOPPER_KICKER_OMNI_CAN_ID);
        feederBelts = new VictorSPX(RobotMap.HOPPER_FEEDER_BELTS_CAN_ID);

        setBrakeMode(true);
    }

    public void setHopperSpeed(double speed){
        if(speed != 0.0){
            HopperMotor.set(ControlMode.PercentOutput, speed);
            isLoading = true;
        }
        else{
            HopperMotor.set(ControlMode.PercentOutput, 0);
            isLoading = false;
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

    public void stopHopper(){
        setHopperSpeed(0.0);
        setFeederOmniSpeed(0.0);
        setKickerOmniSpeed(0.0);
        setFeederBeltsSpeed(0.0);
    }

    public double getHopperCurrent(){ return HopperMotor.getStatorCurrent(); }

    public double getFeederOmniCurrent(){  return feederOmni.getStatorCurrent(); }

    public void setOverrideStatus(boolean overrideStatus) {
        isOverrode = overrideStatus;
    }
    
    public void setBrakeMode(boolean setBrake){

        if(setBrake){
            HopperMotor.setNeutralMode(NeutralMode.Brake);
            feederOmni.setNeutralMode(NeutralMode.Brake);

            kickerOmni.setNeutralMode(NeutralMode.Brake);
            feederBelts.setNeutralMode(NeutralMode.Brake);

            System.out.println("Hopper Motors in Brake");
        }
        else{
            HopperMotor.setNeutralMode(NeutralMode.Coast);
            feederOmni.setNeutralMode(NeutralMode.Coast);

            kickerOmni.setNeutralMode(NeutralMode.Coast);
            feederBelts.setNeutralMode(NeutralMode.Coast);

            System.out.println("Hopper Motors in Coast");
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
                SmartDashboard.putNumber("Hopper Motor Current: " , HopperMotor.getStatorCurrent());

            } catch (Exception e) {
                System.out.println("Desired Mode Error");
            }
        } else if (mode == DesiredMode.COMPETITION) {

        }
    }
}
