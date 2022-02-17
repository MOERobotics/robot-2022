package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.generic.GenericRobot;

//Simple autonomous code
public class BallCtoBtoTerminal extends GenericAutonomous{
    double startingYaw;
    double startTime;
    double startDistance;

    double leftpower;
    double rightpower;
    double defaultPower = .25;
    double defaultTurnPower = .25;
    double correction;

    double distnacetoC = 37;
    double distancetoB = 169.95;
    double distancetoTerminal = 160.6;
    double angleC = 56.25;
    double angleB = 81.27;
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

        switch (autonomousStep) {
            case 0: //reset
                robot.lowerCollector();
                PIDDriveStraight.reset();
                PIDDriveStraight.enableContinuousInput(-180, 180);
                robot.resetEncoders();
                if (System.currentTimeMillis() - startTime > 100) {
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

                if (robot.getDriveDistanceInchesLeft() - startDistance >= distnacetoC - rampDownDist) {
                    defaultPower = (distnacetoC - robot.getDriveDistanceInchesLeft() + startDistance) * defaultPower / rampDownDist;
                }
                if (robot.getDriveDistanceInchesLeft() - startDistance >= distnacetoC) {
                    autonomousStep += 1;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 5: //stop
                leftpower = 0;
                rightpower = 0;
                if (System.currentTimeMillis() - startTime > 1000) {
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

                if (robot.getYaw() - startingYaw > angleC) {
                    startingYaw = startingYaw + angleC;
                    startDistance = robot.getDriveDistanceInchesLeft();
                    PIDDriveStraight.reset();
                    autonomousStep += 1;
                }
                break;
            case 13: //drive towards the ball B
                correction = PIDDriveStraight.calculate(robot.getYaw() - startingYaw);

                leftpower = defaultPower + correction;
                rightpower = defaultPower - correction;

                if (robot.getDriveDistanceInchesLeft() - startDistance >= distancetoB - rampDownDist) {
                    defaultPower = (distancetoB - robot.getDriveDistanceInchesLeft() + startDistance) * defaultPower / rampDownDist;
                }
                if (robot.getDriveDistanceInchesLeft() - startDistance >= distancetoB) {
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
                leftpower = -defaultTurnPower;
                rightpower = defaultTurnPower;
                //turning ???

                if (robot.getYaw() - startingYaw > angleB) {
                    startingYaw = robot.getYaw();
                    startDistance = robot.getDriveDistanceInchesLeft();
                    autonomousStep += 1;
                }
                break;
            case 22: //drive to ball Terminal
                correction = PIDDriveStraight.calculate(robot.getYaw() - startingYaw);

                leftpower = defaultPower + correction;
                rightpower = defaultPower - correction;

                if (robot.getDriveDistanceInchesLeft() - startDistance >= distancetoTerminal - rampDownDist) {
                    defaultPower = (distancetoTerminal - robot.getDriveDistanceInchesLeft() + startDistance) * defaultPower / rampDownDist;
                }
                if (robot.getDriveDistanceInchesLeft() - startDistance >= distancetoTerminal) {
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
    }
}

