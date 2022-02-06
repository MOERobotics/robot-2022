package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericAutonomous;
import frc.robot.generic.GenericRobot;

//Simple autonomous code for ball A, closest ball to the hangar
public class Basic extends GenericAutonomous {
    double startingYaw;

    int autonomousStep;

    double leftpower;
    double rightpower;
    double defaultPower = .25;

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

                if(robot.getDriveDistanceInchesLeft() < -37){
                    autonomousStep += 1;
                } //has 3 inches of momentum with .25 power
                break;
            case 5: //stop
                leftpower = 0;
                rightpower = 0;
                break;
            case 6: //collector to collect ball
                autonomousStep += 1;
                break;
            case 7: //shoots second ball
                autonomousStep += 1;
                break;
        }
        robot.drivePercent(leftpower, rightpower);

    }
}
