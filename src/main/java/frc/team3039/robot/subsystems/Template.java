package frc.team3039.robot.subsystems;

import frc.team3039.robot.loops.ILooper;
import frc.team3039.robot.AutoRoutineSelector.DesiredMode;

//Implement Loop CLASS SPECIFIC

public class Template extends Subsystem{
    public void writeToLog() {
    }

    // Optional design pattern for caching periodic reads to avoid hammering the
    // HAL/CAN.
    public void readPeriodicInputs() {
    }

    // Optional design pattern for caching periodic writes to avoid hammering the
    // HAL/CAN.
    public void writePeriodicOutputs() {
    }

    public  boolean checkSystem(){
        System.out.println("TESTING *INSERT CLASS NAME* ");
        return false;
    }

    public void outputTelemetry(DesiredMode operationMode) {
        {
            if (operationMode == DesiredMode.TEST) {
                try {


                } catch (Exception e) {
                    System.out.println("Desired Mode Error");
                }
            } else if (operationMode == DesiredMode.COMPETITION) {

            }
        }
    }



    @Override
    public synchronized void stop() {

    }
    public void zeroSensors() {
    }

    public void registerEnabledLoops(ILooper enabledLooper) {
    }
}