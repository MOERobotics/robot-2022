package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;

//Simple autonomous code for ball A, closest ball to the hangar
public class BallBtoTerminal extends GenericAutonomous {
    double startingYaw;
    double leftpower;
    double rightpower;
    double defaultPower = .4;

    double correction;
    double startDistance;
    double startTime;

    PIDController PIDDriveStraight;

    @Override
    public void autonomousInit(GenericRobot robot) {
        autonomousStep = 0;
        startingYaw = robot.getYaw(); //might need to change to set degrees
        PIDDriveStraight = new PIDController(robot.getPIDmaneuverP(), robot.getPIDmaneuverI(), robot.getPIDmaneuverD());
        startTime = System.currentTimeMillis();
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {

        switch(autonomousStep){
            case 0: //reset
                robot.lowerCollector();
                PIDDriveStraight.reset();
                PIDDriveStraight.enableContinuousInput(-180,180);
                robot.resetEncoders();
                robot.resetAttitude();
                if (System.currentTimeMillis() - startTime > 100){
                    startDistance = robot.getDriveDistanceInchesLeft();
                    startingYaw = robot.getYaw();
                    autonomousStep = 4;
                }
                break;
            case 1: //shoot the ball
            case 2: //shoot the ball part 2 electric boogaloo
            case 3: //shoot the ball part 3 maybe
            case 4: //drive to ball A
                correction = PIDDriveStraight.calculate(robot.getYaw() - startingYaw);

                leftpower = defaultPower + correction;
                rightpower = defaultPower - correction;

                if(robot.getDriveDistanceInchesLeft() >= 61.5){
                    autonomousStep += 1;
                    startTime = System.currentTimeMillis();
                } //has 3 inches of momentum with .25 power
                break;
            case 5: //stop
                leftpower = 0;
                rightpower = 0;
                autonomousStep = 12;
                break;
            case 6: //collector to collect ball
            case 7: //collection part 2 not electric nor boogaloo
            case 8: //nother collection case
            case 9: //shoot the second ball for funsies
            case 10: //miss the target and become sadge
            case 11: //copium
                //will change these comments when they actually mean something
            case 12://reset
                if (System.currentTimeMillis() - startTime >= 1000){
                    PIDDriveStraight.reset();
                    PIDDriveStraight.enableContinuousInput(-180,180);
                    startDistance = robot.getDriveDistanceInchesLeft();
                    autonomousStep +=1;
                }
                break;
            case 13://drive to ball at terminal
                correction = PIDDriveStraight.calculate(robot.getYaw() - startingYaw);

                leftpower = defaultPower + correction;
                rightpower = defaultPower - correction;

                if(robot.getDriveDistanceInchesLeft() - startDistance >= 153.5){
                    autonomousStep += 1;
                }
                break;
            case 14:
                leftpower = 0;
                rightpower = 0;
                break;
            case 15: //collector to collect ball
            case 16: //collection part 2 not electric nor boogaloo
            case 17: //nother collection case
            case 18: //shoot the second ball for funsies
        }
        robot.drivePercent(leftpower, rightpower);

    }
}
