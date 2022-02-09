package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericAutonomous;
import frc.robot.generic.GenericRobot;

//Simple autonomous code for ball A, closest ball to the hangar
public class SimpleBTerminal extends GenericAutonomous {
    double startingYaw;

    int autonomousStep;

    double leftpower;
    double rightpower;
    double defaultPower = .4;

    double correction;

    PIDController PIDDriveStraight = new PIDController(.025, 0, 0);

    @Override
    public void autonomousInit(GenericRobot robot) {
        autonomousStep = 0;
        startingYaw = robot.getYaw(); //might need to change to set degrees

    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {
        SmartDashboard.putNumber("Autonomous Step", autonomousStep);
        SmartDashboard.putNumber("Position", robot.getDriveDistanceInchesLeft());
        SmartDashboard.putNumber("Starting Yaw", startingYaw);
        SmartDashboard.putNumber("Current Yaw", robot.getYaw());

        switch(autonomousStep){
            case 0: //reset
                PIDDriveStraight.reset();
                PIDDriveStraight.enableContinuousInput(-180,180);
                robot.resetEncoders();
                autonomousStep = 4;
                break;
            case 1: //shoot the ball
            case 2: //shoot the ball part 2 electric boogaloo
            case 3: //shoot the ball part 3 maybe
            case 4: //drive to ball A
                correction = PIDDriveStraight.calculate(robot.getYaw() - startingYaw);

                leftpower = defaultPower - correction;
                rightpower = defaultPower + correction;

                if(robot.getDriveDistanceInchesLeft() < -61.5){
                    autonomousStep += 1;
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
                //will change these comments when they actually mean somthing
            case 12://reset
                PIDDriveStraight.reset();
                PIDDriveStraight.enableContinuousInput(-180,180);
                robot.resetEncoders();
                startingYaw = robot.getYaw();
                autonomousStep +=1;
                break;
            case 13://reset
                correction = PIDDriveStraight.calculate(robot.getYaw() - startingYaw);

                leftpower = defaultPower - correction;
                rightpower = defaultPower + correction;

                if(robot.getDriveDistanceInchesLeft() < -153.5){
                    autonomousStep += 1;
                } //has 3 inches of momentum with .25 powercase 5: //stop\
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
