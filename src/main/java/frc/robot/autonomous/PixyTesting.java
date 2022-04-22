package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.command.PixyAutoTrack;
import frc.robot.generic.GenericRobot;

public class PixyTesting extends GenericAutonomous {

    double startEncoder;
    long startTime;
    PIDController PIDDriveStraight;

    double cameraConnection = 0;

    double highPower = .6;
    double travelDist = 140;
    double yawTolerance = 10;

    double centerYaw;

    @Override
    public void autonomousInit(GenericRobot robot){
        autonomousStep = 0;
        startEncoder = robot.getDriveDistanceInchesLeft();
        PIDDriveStraight = new PIDController(robot.getPIDmaneuverP(), robot.getPIDmaneuverI(), robot.getPIDmaneuverD());
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
                // Update cameraOffset based on pixycam
                double tolerance = 0.15;
                double cameraSteer = 1.5;
                double offset = robot.pixyOffsetOfClosest();
                if(offset > tolerance){
                    cameraConnection += cameraSteer;
                } else if(offset < -tolerance){
                    cameraConnection -= cameraSteer;
                }
                // Correct by up to 10deg
                cameraConnection = Math.min(Math.max(cameraConnection, -10), 10);

                if(robot.getDriveDistanceInchesLeft() < 9){
                    cameraConnection = 0;
                }

                SmartDashboard.putNumber("Camera Connection", cameraConnection);
                double targetYaw = centerYaw + cameraConnection;

                double correction = PIDDriveStraight.calculate(robot.getYaw() - targetYaw);

                leftPower = highPower + correction;
                rightPower = highPower - correction;


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
