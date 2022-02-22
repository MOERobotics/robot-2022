package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;

//Simple autonomous code for ball A, and then driving to the ball B, and finally to ball at terminal
public class BallAtoBtoTerminal extends GenericAutonomous {
    double startingYaw;
    double startTime;
    double startDistance;

    double leftpower;
    double rightpower;
    double defaultPower = .4;
    double defaultTurnPower = .4;
    double correction;

    double distanceA = 37;
    double distanceB = 117.1;
    double distanceTerminal = 160.6;
    double angleA = 87.74;
    double angleB = 0; //ASK CAD LATER
    double rampDownDist = 10;

    PIDController PIDDriveStraight;

    @Override
    public void autonomousInit(GenericRobot robot) {
        autonomousStep = 0;
        startingYaw = robot.getYaw();
        startTime = System.currentTimeMillis();
        PIDDriveStraight = new PIDController(robot.getPIDmaneuverP(), robot.getPIDmaneuverI(), robot.getPIDpivotD());
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {

        switch(autonomousStep){
            case 0: //reset
                robot.lowerCollector();
                PIDDriveStraight.reset();
                PIDDriveStraight.enableContinuousInput(-180,180);
                robot.resetEncoders();
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

                if(robot.getDriveDistanceInchesLeft() - startDistance >= distanceA - rampDownDist){
                    defaultPower = (distanceA-robot.getDriveDistanceInchesLeft()+startDistance)*defaultPower/rampDownDist;
                }
                if(robot.getDriveDistanceInchesLeft() - startDistance >= distanceA){
                    autonomousStep += 1;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 5: //stop
                leftpower = 0;
                rightpower = 0;
                if (System.currentTimeMillis() - startTime > 1000){
                    autonomousStep = 12;
                }
                break;
            case 6: //collector to collect ball
            case 7: //collection part 2 not electric nor boogaloo
            case 8: //nother collection case
            case 9: //shoot the second ball for funsies
            case 10: //miss the target and become sadge
            case 11: //copium
                //will change these comments when they actually mean something
            case 12: //turn to go to ball B
                leftpower = defaultTurnPower;
                rightpower = -defaultTurnPower;
                //turning right

                if(robot.getYaw() - startingYaw > angleA) {
                    startingYaw = startingYaw + angleA;
                    startDistance = robot.getDriveDistanceInchesLeft();
                    PIDDriveStraight.reset();
                    autonomousStep += 1;
                }
                break;
            case 13: //drive towards the ball B
                correction = PIDDriveStraight.calculate(robot.getYaw() - startingYaw);

                leftpower = defaultPower + correction;
                rightpower = defaultPower - correction;

                if(robot.getDriveDistanceInchesLeft() - startDistance >= distanceB - rampDownDist){
                    defaultPower = (distanceB-robot.getDriveDistanceInchesLeft()+startDistance)*defaultPower/rampDownDist;
                }
                if(robot.getDriveDistanceInchesLeft() - startDistance >= distanceB) {
                    autonomousStep += 1;
                    leftpower = 0;
                    rightpower = 0;
                }
                break;
            case 14: //collect / shoot
            case 15:
            case 16:
            case 17:
            case 18:
            case 19:
            case 20:
            case 21: //turn to ball Terminal
                leftpower  = -defaultTurnPower;
                rightpower = defaultTurnPower;
                //turning ???

                if(robot.getYaw() - startingYaw > angleB) {
                    startingYaw = robot.getYaw();
                    startDistance = robot.getDriveDistanceInchesLeft();
                    autonomousStep += 1;
                }
                break;
            case 22: //drive to ball Terminal
                correction = PIDDriveStraight.calculate(robot.getYaw() - startingYaw);

                leftpower = defaultPower + correction;
                rightpower = defaultPower - correction;

                if(robot.getDriveDistanceInchesLeft() - startDistance >= distanceTerminal - rampDownDist){
                    defaultPower = (distanceTerminal -robot.getDriveDistanceInchesLeft()+startDistance)*defaultPower/rampDownDist;
                }
                if(robot.getDriveDistanceInchesLeft() - startDistance >= distanceTerminal) {
                    autonomousStep += 1;
                    leftpower = 0;
                    rightpower = 0;
                }
                break;
            case 23:
                leftpower = 0;
                rightpower = 0;
                break;
            case 24:
            case 25:
        }
        robot.drivePercent(leftpower, rightpower);

    }
}