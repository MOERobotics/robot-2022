package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;

//insert blurb
public class autoArc extends GenericAutonomous {
    double rollout = 72; //center of rotation changes per robot. Test value per robot
    double radius = 153;
    double wheelBase = 28;
    double innerRadius = radius - wheelBase/2;
    double outerRadius = radius + wheelBase/2;
    double outerArcDist = 180;
    double defaultPower = .4;
    double pivotDeg = 90;
    double rampDownDist = 10;

    double startYaw;
    double currentYaw;
    double startInches;
    double currentDistInches;
    double startingTime;
    double leftPower;
    double rightPower;

    double correction;
    PIDController PIDSteering;
    PIDController PIDPivot;
    PIDController turretPIDController;

    boolean time = false;
    int counter;
    double[] averageX = new double [2];
    double currentTurretPower = 0;
    double average;
    int averageTurretXSize = 2;
    boolean breakRobot = false;
    boolean startTimer = false;

    @Override
    public void autonomousInit(GenericRobot robot) {
        startingTime        = System.currentTimeMillis();
        PIDSteering         = new PIDController(
            robot.getPIDmaneuverP(),
            robot.getPIDmaneuverI(),
            robot.getPIDmaneuverD()
        );
        PIDPivot            = new PIDController(
            robot.getPIDpivotP(),
            robot.getPIDpivotI(),
            robot.getPIDpivotD()
        );
        turretPIDController = new PIDController(
            robot.turretPIDgetP(),
            robot.turretPIDgetI(),
            robot.turretPIDgetD()
        );
        defaultPower = .4;
        autonomousStep = 0;
        time = false;
        counter = 0;
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {

        if (autonomousStep >= 1){
            robot.getCargo();
            robot.shoot();
            robot.setTurretPitchPosition(.38);
        }
        switch (autonomousStep){
            case 0: //reset
                robot.lowerCollector();
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
                if (currentDistInches - startInches >= rollout-rampDownDist){ //slowdown
                    double a = rampDown(defaultPower, 0, startInches, rampDownDist, currentDistInches, rollout);
                    leftPower = a;
                    rightPower = a;
                }
                if (currentDistInches - startInches >= rollout){
                    leftPower = 0;
                    rightPower = 0;
                    autonomousStep += 1;
                }
                break;
            case 2:
                if (robot.canShoot() && robot.isTargetFound() &&(-5<average) && (average < 5)){
                    startingTime = System.currentTimeMillis();
                    robot.setActivelyShooting(true);
                    autonomousStep += 1.0;
                }
                break;
            case 3:
                if (System.currentTimeMillis() - startingTime >= 1000){
                    robot.setActivelyShooting(true);
                    robot.setShooterTargetRPM(500); //super low power to yeet out undesirable cargo
                    autonomousStep += 1;
                }
                break;
            case 4: // Pid pivot 90 degrees ccw
                currentYaw = robot.getYaw();
                correction = PIDPivot.calculate(pivotDeg + currentYaw - startYaw);
                leftPower = correction;
                rightPower = -correction; //TODO: on lightning, change to abs encoder
                if (!robot.isTargetFound() && !breakRobot) {
                    startingTime = System.currentTimeMillis();
                    breakRobot = true;
                }

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

            case 5: //Pid reset
                PIDSteering.reset();
                PIDPivot.reset();
                PIDSteering.disableContinuousInput();
                PIDPivot.enableContinuousInput(-180, 180);
                startYaw = startYaw - pivotDeg;
                startInches = robot.getDriveDistanceInchesRight();
                autonomousStep += 1;
                break;

            case 6: //Pid Arc 10 ft Left
                currentYaw = robot.getYaw();
                currentDistInches = robot.getDriveDistanceInchesRight();
                correction = PIDSteering.calculate(Math.toDegrees((currentDistInches-startInches)/outerRadius) + (currentYaw-startYaw));
                double radiusRatio = outerRadius/innerRadius;
                leftPower = 1/Math.sqrt(radiusRatio)*defaultPower + correction;
                rightPower = (Math.sqrt(radiusRatio))*defaultPower - correction;
                if (currentDistInches - startInches >= outerArcDist){
                    leftPower = 0;
                    rightPower = 0;
                    autonomousStep += 1;
                }
                if (currentDistInches - startInches >= outerArcDist - 24){
                    robot.setShooterTargetRPM(5000); //set it back to where we want
                    robot.setActivelyShooting(false);
                }
                break;

            case 7: //stop
                leftPower = 0;
                rightPower = 0;
                autonomousStep += 1;
                break;
            case 8:   //shoot last cargo
                if (robot.canShoot()){
                    robot.setActivelyShooting(true);
                }
                break;


        }
        robot.drivePercent(leftPower, rightPower);

        if(robot.isTargetFound()) {
            averageX[counter % averageTurretXSize] = robot.getTargetX();
            counter++;
        }

        average = 0;
        for(double i: averageX){
            average += i;
        }
        average /= averageTurretXSize;

        currentTurretPower = 0;
        if(robot.isTargetFound()){
            currentTurretPower = turretPIDController.calculate(average);
        }else{
            turretPIDController.reset();
        }
        if((!robot.isTargetFound()) && (System.currentTimeMillis() - startingTime < 2000)) {
            currentTurretPower = .2;
        }
        robot.setTurretPowerPct(currentTurretPower);

    }

}
