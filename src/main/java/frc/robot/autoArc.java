package frc.robot;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericAutonomous;
import frc.robot.generic.GenericRobot;

public class autoArc extends GenericAutonomous {
    double rollout = 72; //center of rotation changes per robot. Test value per robot
    double radius = 153;
    double wheelBase = 28;
    double innerRadius = radius - wheelBase/2;
    double outerRadius = radius + wheelBase/2;
    double outerArcDist = 180;

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
    PIDController PIDSteering;
    PIDController PIDPivot;

    boolean time = false;

    @Override
    public void autonomousInit(GenericRobot robot) {
        startingTime = System.currentTimeMillis();
        PIDSteering = new PIDController(robot.getPIDmaneuverP(), robot.getPIDmaneuverI(), robot.getPIDmaneuverD());
        PIDPivot = new PIDController(robot.getPIDpivotP(), robot.getPIDpivotI(), robot.getPIDpivotD());
        defaultPower = .4;
        autonomousStep = 0;
        time = false;
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {
        SmartDashboard.putNumber("autonomousStep", autonomousStep);
        SmartDashboard.putNumber("correction", correction);
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
                    defaultPower = (rollout-currentDistInches+startInches)*.04;
                }
                if (currentDistInches - startInches >= rollout){
                    defaultPower = .4;
                    leftPower = 0;
                    rightPower = 0;
                    autonomousStep += 1;
                }
                break;

            case 2: // Pid pivot 90 degrees ccw
                currentYaw = robot.getYaw();
                correction = PIDPivot.calculate(pivotDeg + currentYaw - startYaw);
                leftPower = correction;
                rightPower = -correction;
                if (Math.abs(Math.abs(currentYaw - startYaw)-pivotDeg) <= 1.5){
                    if (!time){
                        startingTime = System.currentTimeMillis();
                        time = true;
                    }
                }
                else{
                    startingTime = System.currentTimeMillis();
                    time = false;
                }
                if (System.currentTimeMillis() - startingTime >= 50){
                    leftPower = 0;
                    rightPower = 0;
                    autonomousStep += 1;
                    startingTime = System.currentTimeMillis();
                }

                break;

            case 3: //Pid reset
                PIDSteering.reset();
                PIDPivot.reset();
                PIDSteering.disableContinuousInput();
                PIDPivot.enableContinuousInput(-180, 180);
                startYaw = startYaw - pivotDeg;
                startInches = robot.getDriveDistanceInchesRight();
                autonomousStep += 1;
                break;

            case 4: //Pid Arc 10 ft Left
                currentYaw = robot.getYaw();
                currentDistInches = robot.getDriveDistanceInchesRight();
                SmartDashboard.putNumber("startInches", startInches);
                SmartDashboard.putNumber("currentInches", currentDistInches);
                SmartDashboard.putNumber("outerRadius", outerRadius);
                SmartDashboard.putNumber("startYaw", startYaw);
                SmartDashboard.putNumber("currentYaw", currentYaw);
                SmartDashboard.putNumber("InchtoDeg", Math.toDegrees((currentDistInches-startInches)/outerRadius));
                SmartDashboard.putNumber("correction Measurement", Math.toDegrees((currentDistInches-startInches)/outerRadius) + (currentYaw-startYaw));
                correction = PIDSteering.calculate(Math.toDegrees((currentDistInches-startInches)/outerRadius) + (currentYaw-startYaw));
                double radiusRatio = outerRadius/innerRadius;
                leftPower = 1/Math.sqrt(radiusRatio)*defaultPower + correction;
                rightPower = (Math.sqrt(radiusRatio))*defaultPower - correction;
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
