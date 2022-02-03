package frc.robot;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericAutonomous;
import frc.robot.generic.GenericRobot;

public class autoArc extends GenericAutonomous {
    double rollout = 120;
    double radius = 153;
    double wheelBase = 28;
    double innerRadius = radius - wheelBase/2;
    double outerRadius = radius + wheelBase/2;
    double outerArcDist = 120;

    int autonomousStep = 0;

    double startYaw;
    double currentYaw;
    double pivotDeg = 90;

    double startInches;
    double currentDistInches;

    double startingTime;

    double leftPower;
    double rightPower;
    double defaultPower = .4;

    double correction;

    @Override
    public void autonomousInit(GenericRobot robot) {
        startingTime = System.currentTimeMillis();
        autonomousStep = -1;
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {
        PIDController PIDSteering = new PIDController(robot.getPIDmaneuverP(), robot.getPIDmaneuverI(), robot.getPIDmaneuverD());
        PIDController PIDPivot = new PIDController(robot.getPIDpivotP(), robot.getPIDpivotI(), robot.getPIDpivotD());
        switch (autonomousStep){
            case 0: //reset
                PIDSteering.reset();
                PIDPivot.reset();
                PIDSteering.enableContinuousInput(-180,180);
                PIDPivot.enableContinuousInput(-180,180);
                robot.resetAttitude();
                robot.resetEncoders();
                startYaw = robot.getYaw();
                startInches = robot.getDriveDistanceInchesLeft();
                if (System.currentTimeMillis() - startingTime >= 100){
                    autonomousStep += 1;
                }
                break;

            case 1: //straightaway
                currentYaw = robot.getYaw();
                currentDistInches = robot.getDriveDistanceInchesLeft();
                correction = PIDSteering.calculate(currentYaw - startYaw);
                leftPower = defaultPower + correction;
                rightPower = defaultPower - correction;
                if (currentDistInches - startInches >= rollout-10){ //slowdown
                    defaultPower = .2;
                }
                if (currentDistInches - startInches >= rollout){
                    defaultPower = .2;
                    leftPower = 0;
                    rightPower = 0;
                    autonomousStep = 5;
                }
                break;

            case 2: // Pid pivot 90 degrees ccw
                currentYaw = robot.getYaw();
                correction = PIDPivot.calculate(pivotDeg + currentYaw - startYaw);
                leftPower = defaultPower - correction;
                rightPower = defaultPower + correction;
                if (currentYaw - startYaw <= -pivotDeg){
                    leftPower = 0;
                    rightPower = 0;
                    autonomousStep += 1;
                }
                break;

            case 3: //Pid reset
                PIDSteering.reset();
                PIDPivot.reset();
                PIDSteering.enableContinuousInput(-180,180);
                PIDPivot.enableContinuousInput(-180,180);
                robot.resetAttitude();
                robot.resetEncoders();
                startYaw = startYaw - 90;
                startInches = robot.getDriveDistanceInchesLeft();
                autonomousStep += 1;
                break;

            case 4: //Pid Arc 10 ft Left
                currentYaw = robot.getYaw();
                currentDistInches = robot.getDriveDistanceInchesLeft();
                correction = PIDSteering.calculate(currentDistInches-startInches - Math.toRadians(currentYaw-startYaw)*outerRadius);
                double radiusRatio = outerRadius/innerRadius;
                leftPower = Math.sqrt(radiusRatio)*defaultPower - correction;
                rightPower = 1/(Math.sqrt(radiusRatio))*defaultPower + correction;
                if (currentDistInches - startInches >= outerArcDist){
                    leftPower = 0;
                    rightPower = 0;
                    autonomousStep += 1;
                }
                break;

            case 5: //stop
                leftPower = 0;
                rightPower = 0;
                break;

        }
        robot.drivePercent(leftPower, rightPower);

    }

}
