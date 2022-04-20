package frc.robot.command;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.generic.GenericRobot;
import frc.robot.generic.Pixycam;

public class PixyAutoTrack {

    PIDController PIDdrive;

    double cameraCorrection = 0;
    double mostRecentAbsOffset = 0;
    int mostRecentBlockCount = 0;

    //bounds for the distances pixy track is active
    double pixyDistNear = 18;
    double pixyDistFar = 72;

    static double pixyToleranceNarrow = 0.1;
    static double pixyToleranceWide = 0.3;
    static double steerConstant = 2; //steer if robot is at 100% power
    static double deviationLimit = 20;

    public PixyAutoTrack(PIDController pid){
        PIDdrive = pid;
    }

    public void resetCorrection(){
        cameraCorrection = 0;
    }

    public void updateReqCorrection(GenericRobot robot, double power, double centerYaw) {
        updateReqCorrection(robot, power, centerYaw, false, 42);
    }

    //power(speed) is important because higher power means higher PID strength needed
    public void updateReqCorrection(GenericRobot robot, double power, double centerYaw, boolean lockOnTarget, int targetID){
        double offset = 0;

        //if we have a lock on, search for that specific cargo
        if(lockOnTarget){
            Pixycam pixycam = robot.getPixyCam();
            if(pixycam != null){
                Pixycam.PixyCargo[] pixycargos = pixycam.getCargo(false);
                for(Pixycam.PixyCargo pixyCargo : pixycargos){
                    if(pixyCargo.getId() == targetID){
                        offset = pixyCargo.getProportionalOffsetY();
                    }
                }
            }
        }
        else{
            offset = robot.pixyOffsetOfClosest();
        }

        mostRecentBlockCount = robot.pixyCargoCount();
        double absoluteOS = Math.abs(offset);
        mostRecentAbsOffset = absoluteOS;

        double smoothingConstant = 0;
        if(absoluteOS > pixyToleranceWide){
            smoothingConstant = 1;
        }
        else if(absoluteOS > pixyToleranceNarrow){
            smoothingConstant = (absoluteOS - pixyToleranceNarrow) / (pixyToleranceWide - pixyToleranceNarrow);
        }

        if(offset > 0) {
            cameraCorrection += smoothingConstant * steerConstant * power;
        }
        else{
            cameraCorrection -= smoothingConstant * steerConstant * power;
        }

        if(absoluteOS < pixyToleranceNarrow){
            cameraCorrection = robot.getYaw() - centerYaw;
        }

        // Correct by up to x deg: clamp
        cameraCorrection = Math.min(Math.max(cameraCorrection, -deviationLimit), deviationLimit);
    }


    public double getPIDCorrection(GenericRobot robot, double centerYaw){
        double targetYaw = centerYaw + cameraCorrection;
        double correction = PIDdrive.calculate(robot.getYaw() - targetYaw);

        return correction;
    }

    public double getPixyDistNear(){
        return pixyDistNear;
    }
    public double getPixyDistFar(){
        return pixyDistFar;
    }

    public double getMostRecentAbsOffset() { return mostRecentAbsOffset; }
    public int getMostRecentBlockCount() { return mostRecentBlockCount; }

    public void setDeviationLimit(double newDeviationLimit){
        deviationLimit = newDeviationLimit;
    }
}