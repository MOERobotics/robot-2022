package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.generic.GenericRobot;

//Setup: put the robot down so that the ball at the terminal and the ball B are lined up straight.
public class BallSimpleB extends GenericAutonomous {
    double startingYaw;

    double startDistance;
    double leftpower;
    double rightpower;
    double defaultPower = .4;
    double correction;
    double startTime;

    double distanceB = 61.5; //distance to ball B from the tarmac
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
                    autonomousStep = 4;
                    startingYaw = robot.getYaw();
                    startDistance = robot.getDriveDistanceInchesLeft();
                }
                break;
            case 1: //shoot the ball
            case 2: //shoot the ball part 2 electric boogaloo
            case 3: //shoot the ball part 3 maybe
            case 4: //drive to ball A
                correction = PIDDriveStraight.calculate(robot.getYaw() - startingYaw);

                leftpower = defaultPower + correction;
                rightpower = defaultPower - correction;

                if(robot.getDriveDistanceInchesLeft() - startDistance >= distanceB - rampDownDist){
                    defaultPower = (distanceB - robot.getDriveDistanceInchesLeft() + startDistance)
                            * defaultPower / rampDownDist;
                }
                if(robot.getDriveDistanceInchesLeft() - startDistance >= distanceB){
                    autonomousStep += 1;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 5: //stop
                leftpower = 0;
                rightpower = 0;
                startDistance = robot.getDriveDistanceInchesLeft();
                autonomousStep = 12;
                break;
            case 6: //collect/shoot the ball again
        }
        robot.drivePercent(leftpower, rightpower);
    }


}
