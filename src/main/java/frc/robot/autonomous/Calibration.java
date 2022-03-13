package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.generic.GenericRobot;

//calibrating tool for encoder ratio of our bots, inches/ticks
public class Calibration extends GenericAutonomous {
    double startingYaw;

    double startDistance;
    double leftpower;
    double rightpower;
    double defaultPower = .3;
    double correction;
    double startTime;

    double distance = 72;
    double rampDownDist = 10;

    PIDController PIDDriveStraight;

    @Override
    public void autonomousInit(GenericRobot robot){
        autonomousStep = 0;
        startingYaw = robot.getYaw();
        startTime = System.currentTimeMillis();
        PIDDriveStraight = new PIDController(robot.getPIDmaneuverP(), robot.getPIDmaneuverI(), robot.getPIDmaneuverD());
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot){

        switch(autonomousStep){
            case 0: //reset
                robot.lowerCollector();
                PIDDriveStraight.reset();
                PIDDriveStraight.enableContinuousInput(-180,180);
                robot.resetEncoders();
                robot.resetAttitude();
                if (System.currentTimeMillis() - startTime > 100){
                    autonomousStep += 1;
                    startingYaw = robot.getYaw();
                    startDistance = robot.getDriveDistanceInchesLeft();
                }
                break;
            case 1: //drive forward 6ft
                correction = PIDDriveStraight.calculate(robot.getYaw() - startingYaw);


                leftpower = defaultPower + correction;
                rightpower = defaultPower - correction;

                if(robot.getDriveDistanceInchesLeft() - startDistance >= distance - rampDownDist){
                    double ramp = rampDown(defaultPower, .1, startDistance, rampDownDist,
                            robot.getDriveDistanceInchesLeft(), distance);
                    leftpower = ramp;
                    rightpower = ramp;
                }
                if(robot.getDriveDistanceInchesLeft() - startDistance >= distance){
                    autonomousStep += 1;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 2: //stop
                leftpower = 0;
                rightpower = 0;
                break;
        }
        robot.drivePercent(leftpower, rightpower);
    }
}
