package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;

//Simple autonomous code for ball C, closest ball to the hangar, and driving to the ball at terminal
public class BallCtoTerminal extends GenericAutonomous {
    double startingYaw;

    double startDistance;
    double leftpower;
    double rightpower;
    double defaultPower = .4;
    double defaultTurnPower = .4;
    double correction;
    double startTime;

    double distanceC = 40.44;
    double distanceTerminal = 251;
    double angleC = 84.54;
    double rampDownDist = 10;

    PIDController PIDDriveStraight;

    TurretTracker tracker = new TurretTracker();

    @Override
    public void autonomousInit(GenericRobot robot) {
        autonomousStep = 0;
        startingYaw = robot.getYaw();
        startTime = System.currentTimeMillis();
        PIDDriveStraight = new PIDController(robot.getPIDmaneuverP(), robot.getPIDmaneuverI(), robot.getPIDmaneuverD());
        tracker.turretInit(robot);
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {
        tracker.turretUpdate(robot);

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

                if(robot.getDriveDistanceInchesLeft() - startDistance >= distanceC - rampDownDist){
                    defaultPower = (distanceC-robot.getDriveDistanceInchesLeft()+startDistance)
                            *defaultPower/rampDownDist;
                }
                if(robot.getDriveDistanceInchesLeft() - startDistance >= distanceC){
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
            case 6: //collector to collect ball
            case 7: //collection part 2 not electric nor boogaloo
            case 8: //another collection case
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
            case 13: //turn to go to ball @ terminal
                leftpower = -defaultTurnPower;
                rightpower = defaultTurnPower;
                //turning left

                if(robot.getYaw() - startingYaw < -angleC) {
                    startingYaw = startingYaw - angleC;
                    startDistance = robot.getDriveDistanceInchesLeft();
                    PIDDriveStraight.reset();
                    autonomousStep += 1;
                }
                break;
            case 14: //drive towards the ball
                correction = PIDDriveStraight.calculate(robot.getYaw() - startingYaw);

                leftpower = defaultPower + correction;
                rightpower = defaultPower - correction;

                if(robot.getDriveDistanceInchesLeft() - startDistance >= distanceTerminal - rampDownDist){
                    defaultPower = (distanceTerminal-robot.getDriveDistanceInchesLeft()+startDistance)
                            *defaultPower/rampDownDist;
                }
                if(robot.getDriveDistanceInchesLeft() - startDistance >= distanceTerminal){
                    autonomousStep += 1;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 15:
                leftpower = 0;
                rightpower = 0;
                break;
        }
        robot.drivePercent(leftpower, rightpower);
        tracker.turretMove(robot);
    }
}
