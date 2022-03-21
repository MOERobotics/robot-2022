package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;

//Simple autonomous code for ball A, closest ball to the scoring table, and driving to the ball at terminal
//Setup: Line the robot straight on the tape facing ball A
public class BallAAutoArc5Ball extends GenericAutonomous {
    double startingYaw;
    double startTime;
    double startDistance;

    double leftpower;
    double rightpower;
    double defaultPower = .4;
    double correction;
    boolean time = false;
    boolean readyToShoot = false;
    double shooterTargetRPM;

    int averageTurretXSize = 2;
    double[] averageTurretX = new double [averageTurretXSize];
    int counter = 0;

    double distanceA = 44.2;
    double backTrack = 24.5;
    double distanceTerminal = 259.26;
    double radiusA = 247;
    double radiusB = 172.63;
    double angleA = 123;
    double angleB = 62;
    double angleC = 56.067;
    double rampDownDist = 10;

    double radius = radiusA;
    double outerRadius = radius + 14;
    double innerRadius = radius - 14;

    double currentYaw;
    double currentDistInches;

    PIDController PIDDriveStraight;
    PIDController PIDPivot;
    PIDController turretPIDController;
    @Override
    public void autonomousInit(GenericRobot robot) {
        autonomousStep = 0;
        startingYaw = robot.getYaw();
        startTime = System.currentTimeMillis();
        shooterTargetRPM = 0;
        readyToShoot = false;
        PIDDriveStraight = new PIDController(robot.getPIDmaneuverP(), robot.getPIDmaneuverI(), robot.getPIDpivotD());
        PIDPivot = new PIDController(robot.getPIDpivotP(), robot.getPIDpivotI(), robot.getPIDpivotD());
        turretPIDController = new PIDController(robot.turretPIDgetP(), robot.turretPIDgetI(), robot.turretPIDgetD());
        robot.setPipeline(0);
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {
        double currentTurretPower = 0;
        double average = 0;

        if(robot.isTargetFound()) {
            averageTurretX[counter % averageTurretXSize] = robot.getTargetX();
            counter++;
        }
        for(double i: averageTurretX){
            average += i;
        }
        average /= averageTurretXSize;

        currentTurretPower = turretPIDController.calculate(average);

        if (autonomousStep < 4){
            if((!robot.isTargetFound()) && (System.currentTimeMillis() - startTime < 5000)) {
                currentTurretPower = .3;
            }
        }
        if ((autonomousStep>=4) && (autonomousStep < 11)){
            if((!robot.isTargetFound()) && (System.currentTimeMillis() - startTime < 5000)) {
                currentTurretPower = .2;
            }
        }


        if (autonomousStep >= 1){
            robot.getCargo();
            robot.shoot();
            robot.setShooterTargetRPM(3700);
        }
        if (autonomousStep >= 1 && autonomousStep <=10){
            robot.setTurretPitchPosition(.38);
        } else{
            robot.setCollectorIntakePercentage(0);
            currentTurretPower = 0;
        }

        robot.setTurretPowerPct(currentTurretPower);

        switch(autonomousStep){
            case 0: //reset
                robot.lowerCollector();
                PIDDriveStraight.reset();
                PIDPivot.reset();
                PIDDriveStraight.enableContinuousInput(-180,180);
                PIDPivot.enableContinuousInput(-180,180);
                robot.resetEncoders();
                robot.resetAttitude();
                if (System.currentTimeMillis() - startTime > 1000){
                    autonomousStep += 1;
                    startingYaw = robot.getYaw();
                    startDistance = robot.getDriveDistanceInchesLeft();
                }
                break;
            case 1: //drive to ball A

                correction = PIDDriveStraight.calculate(robot.getYaw() - startingYaw);

                leftpower = defaultPower + correction;
                rightpower = defaultPower - correction;

                if(robot.getDriveDistanceInchesLeft() - startDistance >= distanceA - rampDownDist){
                    double ramp = rampDown(defaultPower, .1, startDistance, rampDownDist,
                            robot.getDriveDistanceInchesLeft(), distanceA);
                    leftpower = ramp;
                    rightpower = ramp;
                }
                if(robot.getDriveDistanceInchesLeft() - startDistance >= distanceA){
                    leftpower = 0;
                    rightpower = 0;
                    autonomousStep += 1;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 2: //stop
                leftpower = 0;
                rightpower = 0;
                startDistance = robot.getDriveDistanceInchesLeft();
                autonomousStep += 1;
                time = false;
                break;
            case 3: //shoot the ball if target is found
                if (robot.isTargetFound() && robot.canShoot() && (-5 < average) && (average< 5)){
                    robot.setActivelyShooting(true);
                    startTime = System.currentTimeMillis();
                    autonomousStep += 1;
                }
                break;
            case 4: //turn shooter off
                if (System.currentTimeMillis()-startTime >= 2000){
                    robot.setActivelyShooting(false);
                    autonomousStep += 1;
                }
                break;
            case 5: //reset
                PIDDriveStraight.reset();
                PIDDriveStraight.enableContinuousInput(-180,180);
                startDistance = robot.getDriveDistanceInchesLeft();
                autonomousStep +=1;
                break;
            case 6: //go back
                correction = PIDDriveStraight.calculate(robot.getYaw() - startingYaw);

                leftpower = -1*(defaultPower + correction);
                rightpower = -1*(defaultPower - correction);

                if(robot.getDriveDistanceInchesLeft() - startDistance >= backTrack - rampDownDist){
                    double ramp = rampDown(defaultPower, .1, startDistance, rampDownDist,
                            robot.getDriveDistanceInchesLeft(), backTrack);
                    leftpower = -1*(ramp + correction);
                    rightpower = -1*(ramp - correction);
                }
                if(robot.getDriveDistanceInchesLeft() - startDistance >= distanceA){
                    leftpower = 0;
                    rightpower = 0;
                    autonomousStep += 1;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 7: //turn to go to ballB
                correction = PIDPivot.calculate(-angleA + robot.getYaw() - startingYaw);
                leftpower = correction;
                rightpower = -correction;
                robot.setPipeline(1);
                //turning right
                if (Math.abs(Math.abs(robot.getYaw() - startingYaw)-angleA) <= 1.5){
                    if (!time){
                        startTime = System.currentTimeMillis();
                        time = true;
                    }
                }
                else{
                    startTime = System.currentTimeMillis();
                    time = false;
                }
                if (System.currentTimeMillis() - startTime >= 50){
                    leftpower = 0;
                    rightpower = 0;
                    autonomousStep += 1;
                    startingYaw = angleA;
                    startTime = System.currentTimeMillis();
                    startDistance = robot.getDriveDistanceInchesRight();
                }
                break;
            case 8: //PID reset
                PIDDriveStraight.reset();
                PIDPivot.reset();
                PIDDriveStraight.disableContinuousInput();
                PIDPivot.enableContinuousInput(-180, 180);
                startingYaw = angleA;
                startDistance = robot.getDriveDistanceInchesRight();
                radius = radiusA;
                autonomousStep += 1;
                break;
            case 9: //drive towards BallB
                double a = defaultPower;
                if (Math.abs(currentYaw - startingYaw) >= angleB - 15) {
                    double ramp = rampDown(defaultPower, .1, startingYaw, 15, currentYaw, angleB);
                    a = ramp;
                }
                currentYaw = robot.getYaw();
                currentDistInches = robot.getDriveDistanceInchesRight();
                correction = PIDDriveStraight.calculate(Math.toDegrees((currentDistInches-startDistance/outerRadius) + (currentYaw-startingYaw)));
                double radiusRatio = outerRadius/innerRadius;
                leftpower = 1/Math.sqrt(radiusRatio)*a + correction;
                rightpower = (Math.sqrt(radiusRatio))*a - correction;
                if (Math.abs(currentYaw-startingYaw) >= angleB){
                    leftpower = 0;
                    rightpower = 0;
                    autonomousStep += 1;

                }
                break;
            case 10: //shoot this now
                if (robot.isTargetFound() && robot.canShoot() && (-5 < average) && (average< 5)){
                    robot.setActivelyShooting(true);
                    startTime = System.currentTimeMillis();
                    autonomousStep += 1.0;
                }
                break;
            case 11: //turn shooter off
                if (System.currentTimeMillis()-startTime >= 5000){
                    robot.setActivelyShooting(false);
                    autonomousStep += 1;
                }
                break;
            case 12:
                leftpower = 0;
                rightpower = 0;
                break;
        }
        robot.drivePercent(leftpower, rightpower);
    }
}
