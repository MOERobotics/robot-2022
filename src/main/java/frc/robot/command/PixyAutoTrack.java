package frc.robot.command;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.generic.GenericRobot;

public class PixyAutoTrack {

    PIDController PIDdrive;

    double cameraCorrection = 0;

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

    //power(speed) is important because higher power means higher PID strength needed
    public void updateReqCorrection(GenericRobot robot, double power, double centerYaw){
        double offset = robot.pixyOffsetOfClosest();
        double absoluteOS = Math.abs(offset);

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
}
