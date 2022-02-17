package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;

//Simple autonomous code for ball C, closest ball to the hangar, and driving to the ball at terminal
public class SimpleCTerminal extends GenericAutonomous {
    double startingYaw;
    double startDistance;
    double leftpower;
    double rightpower;
    double defaultPower = .25;
    double defaultTurnPower = .25;

    double correction;
    double startTime;

    PIDController PIDDriveStraight;

    @Override
    public void autonomousInit(GenericRobot robot) {
        autonomousStep = 0;
        startingYaw = robot.getYaw(); //might need to change to set degrees
        startTime = System.currentTimeMillis();
        PIDDriveStraight = new PIDController(robot.getPIDmaneuverP(), robot.getPIDmaneuverI(), robot.getPIDmaneuverD());

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

                if(robot.getDriveDistanceInchesLeft() - startDistance >= 37) {
                    autonomousStep += 1;
                } //has 3 inches of momentum with .25 power
                break;
            case 5: //stop
                leftpower = 0;
                rightpower = 0;
                startDistance = robot.getDriveDistanceInchesLeft();
                autonomousStep = 12;
                //autonomousStep = 8;
                break;
            case 6: //collector to collect ball
            case 7: //collection part 2 not electric nor boogaloo
            case 8: //another collection case
            case 9: //shoot the second ball for funsies
            case 10: //miss the target and become sadge
            case 11: //copium
            //will change these comments when they actually mean something
            case 12: //turn to go to ball @ terminal
                leftpower = -defaultTurnPower;
                rightpower = defaultTurnPower;
                //turning left

                if(robot.getYaw() - startingYaw < -84.54) {
                    startingYaw = startingYaw - 84.54;
                    startDistance = robot.getDriveDistanceInchesLeft();
                    PIDDriveStraight.reset();
                    autonomousStep += 1;
                }
                break;
            case 13: //drive towards the ball
                correction = PIDDriveStraight.calculate(robot.getYaw() - startingYaw);

                leftpower = defaultPower + correction;
                rightpower = defaultPower - correction;

                if(robot.getDriveDistanceInchesLeft() - startDistance >= 251) {
                    autonomousStep += 1;
                    leftpower = 0;
                    rightpower = 0;
                } //might need to tune for momentum
                break;
            case 14:
                leftpower = 0;
                rightpower = 0;
                break;
        }
        robot.drivePercent(leftpower, rightpower);
    }
}
