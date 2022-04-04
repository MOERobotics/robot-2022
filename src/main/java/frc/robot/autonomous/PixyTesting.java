package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.generic.GenericRobot;

public class PixyTesting extends GenericAutonomous {

    double startEncoder;
    long startTime;
    PIDController PIDDriveStraight;



    double travelDist = 140;
    double yawTolerance = 10;

    double centerYaw;

    @Override
    public void autonomousInit(GenericRobot robot){
        autonomousStep = 0;
        startEncoder = robot.getDriveDistanceInchesLeft();
        PIDDriveStraight = new PIDController(robot.getPIDmaneuverP(), robot.getPIDmaneuverI(), robot.getPIDmaneuverD());
        startTime = System.currentTimeMillis();
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot){
        double leftPower = 0;
        double rightPower = 0;

        switch(autonomousStep){
            case 0:
                PIDDriveStraight.reset();
                PIDDriveStraight.enableContinuousInput(-180,180);
                robot.resetEncoders();
                robot.resetAttitude();
                centerYaw = robot.getYaw();

                if (System.currentTimeMillis() - startTime > 100){
                    //startDistance = robot.getDriveDistanceInchesLeft();
                    autonomousStep += 1;
                }
                break;
            case 1:
                leftPower = 0.4;
                rightPower = 0.4;

                double tolerance = 0.3;
                double steer = 0.08;
                double offset = robot.pixyOffsetOfClosest();
                double curYaw = robot.getYaw();
                if(offset > tolerance && curYaw < centerYaw + yawTolerance){
                    leftPower += steer;
                    rightPower -= steer;
                } else if(offset < -tolerance && curYaw > centerYaw - yawTolerance){
                    leftPower -= steer;
                    rightPower += steer;
                }

                if(robot.getDriveDistanceInchesLeft() > startEncoder + travelDist){
                    autonomousStep += 1;
                    leftPower = 0;
                    rightPower = 0;
                }
                break;
            case 2:
                leftPower = 0;
                rightPower = 0;
                break;
        }

        robot.drivePercent(leftPower, rightPower);
    }

}
