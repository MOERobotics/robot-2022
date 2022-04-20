package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.command.PixyAutoTrack;
import frc.robot.generic.GenericRobot;

public class PixyTestingHouston extends GenericAutonomous {

    double startEncoder;
    long startTime;
    PIDController PIDDriveStraight;
    PixyAutoTrack pixyAutoTrack;

    double pixyPower = .3;
    double actualPower = .3;
    double visionDist = 36;
    double travelDist = 36;

    double correction = 0;

    double centerYaw;

    @Override
    public void autonomousInit(GenericRobot robot){
        autonomousStep = 0;
        startEncoder = robot.getDriveDistanceInchesLeft();
        PIDDriveStraight = new PIDController(robot.getPIDmaneuverP(), robot.getPIDmaneuverI(), robot.getPIDmaneuverD());
        pixyAutoTrack = new PixyAutoTrack(PIDDriveStraight);
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
                double distanceTravelled = robot.getDriveDistanceInchesLeft() - startEncoder;

                double startPixyDist = visionDist - pixyAutoTrack.getPixyDistFar();
                double endPixyDist = visionDist - pixyAutoTrack.getPixyDistNear();
                if(distanceTravelled >= startPixyDist && distanceTravelled <= endPixyDist){
                    pixyAutoTrack.updateReqCorrection(robot, pixyPower, centerYaw);
                }

                correction = pixyAutoTrack.getPIDCorrection(robot, centerYaw);

                leftPower = actualPower + correction;
                rightPower = actualPower - correction;

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
