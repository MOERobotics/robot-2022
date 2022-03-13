package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;

//insert blurb
public class AutoArc5Ball extends GenericAutonomous {
    double rollout = 72; //center of rotation changes per robot. Test value per robot
    double radius = 153;
    double wheelBase = 28;
    double innerRadius = radius - wheelBase/2;
    double outerRadius = radius + wheelBase/2;
    double outerArcDist = 180;
    double defaultPower = .4;
    double pivotDeg = 90;
    double rampDownDist = 10;
    double arcAngleA = 163.495;
    double arcAngleB = 71.437;
    double radiusA = 57.077/2;
    double radiusB = 95.079;
    double distanceTerminal = 150; //TODO: go ask cad NICELY for numbers

    double radiusRatio;

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
    int averageTurretXSize = 2;
    boolean breakRobot = false;
    boolean startTimer = false;
    boolean collect = false;

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
        collect = false;
        robot.setPipeline(0);
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {

        if(robot.isTargetFound()) {
            averageX[counter % averageTurretXSize] = robot.getTargetX();
            counter++;
        }

        double average = 0;
        for(double i: averageX){
            average += i;
        }
        average /= averageTurretXSize;

        double currentTurretPower = 0;

        if(robot.isTargetFound()){
            currentTurretPower = turretPIDController.calculate(average);
        }else{
            turretPIDController.reset();
        }


        if((!robot.isTargetFound()) && (System.currentTimeMillis() - startingTime < 5000)) {
            currentTurretPower = .3;
        }
        if ((autonomousStep>=4) && (autonomousStep < 8)){
            if((!robot.isTargetFound()) && (System.currentTimeMillis() - startingTime < 5000)) {
                currentTurretPower = -.2;
            }
        }

        robot.setTurretPowerPct(currentTurretPower);


        if (autonomousStep >= 1){
            if (autonomousStep <= 4){
                robot.getCargo();
            }
            else if (!collect){
                robot.setCollectorIntakePercentage(-1);
            }
            else{
                robot.getCargo();
            }
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
                    double a = rampDown(defaultPower, 0.1, startInches, rampDownDist, currentDistInches, rollout);
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
                    robot.setActivelyShooting(false);
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

                if (Math.abs(Math.abs(currentYaw - startYaw)-pivotDeg) <= 2){
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
                radiusRatio = outerRadius/innerRadius;
                leftPower = 1/Math.sqrt(radiusRatio)*defaultPower + correction;
                rightPower = (Math.sqrt(radiusRatio))*defaultPower - correction;
                if (currentDistInches - startInches >= outerArcDist){
                    leftPower = 0;
                    rightPower = 0;
                    autonomousStep += 1;
                }
                if (currentDistInches - startInches >= outerArcDist - 24){
                    collect = true;
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
                    startingTime = System.currentTimeMillis();
                    autonomousStep += 1;
                }
                break;
            case 9:
                if (System.currentTimeMillis() - startingTime >= 1001){
                    robot.setActivelyShooting(false);
                    autonomousStep += 1;
                }
                break;
            case 10: //PID reset and start arc?
                radius = radiusA;
                PIDSteering.reset();
                PIDPivot.reset();
                PIDSteering.disableContinuousInput();
                PIDPivot.enableContinuousInput(-180, 180);
                startYaw = robot.getYaw();
                startInches = robot.getDriveDistanceInchesLeft();
                autonomousStep += 1;
                break;
            case 11:
                currentYaw = robot.getYaw();
                currentDistInches = robot.getDriveDistanceInchesLeft();
                correction = PIDSteering.calculate(Math.toDegrees((currentDistInches-startInches)/outerRadius) + (currentYaw-startYaw));
                radiusRatio = outerRadius/innerRadius;
                leftPower = (Math.sqrt(radiusRatio))*defaultPower - correction;
                rightPower = 1/Math.sqrt(radiusRatio)*defaultPower + correction;
                if (Math.abs(currentYaw - startYaw) >= arcAngleA){
                    leftPower = 0;
                    rightPower = 0;
                    autonomousStep += 1;
                }
                break;
            case 12:
                radius = radiusB;
                PIDSteering.reset();
                PIDPivot.reset();
                PIDSteering.disableContinuousInput();
                PIDPivot.enableContinuousInput(-180, 180);
                startYaw = robot.getYaw();
                startInches = robot.getDriveDistanceInchesRight();
                autonomousStep += 1;
                break;
            case 13:
                currentYaw = robot.getYaw();
                currentDistInches = robot.getDriveDistanceInchesRight();
                correction = PIDSteering.calculate(Math.toDegrees((currentDistInches-startInches)/outerRadius) + (currentYaw-startYaw));
                radiusRatio = outerRadius/innerRadius;
                leftPower = 1/Math.sqrt(radiusRatio)*defaultPower + correction;
                rightPower = (Math.sqrt(radiusRatio))*defaultPower - correction;
                if (Math.abs(currentYaw - startYaw) >= arcAngleB){
                    leftPower = 0;
                    rightPower = 0;
                    autonomousStep += 1;
                    startingTime = System.currentTimeMillis();
                }
                break;
            case 14:
                PIDSteering.reset();
                PIDPivot.reset();
                PIDSteering.disableContinuousInput();
                PIDPivot.enableContinuousInput(-180, 180);
                startYaw = robot.getYaw();
                startInches = robot.getDriveDistanceInchesLeft();
                autonomousStep += 1;
                if (System.currentTimeMillis() -startingTime >= 2000){
                    autonomousStep += 1;
                }
                //TODO: change the time waited after testing
                break;
            case 15:
                correction = PIDSteering.calculate(robot.getYaw() - startYaw);

                leftPower = -1*(defaultPower - correction);
                rightPower = -1*(defaultPower + correction);

                if(Math.abs(robot.getDriveDistanceInchesLeft() - startInches) >= distanceTerminal - rampDownDist){
                    double ramp = rampDown(defaultPower, 0, startInches, 10,
                            robot.getDriveDistanceInchesLeft(), distanceTerminal);
                    leftPower = -ramp;
                    rightPower = -ramp;
                }
                if(Math.abs(robot.getDriveDistanceInchesLeft() - startInches) >= distanceTerminal){
                    leftPower = 0;
                    rightPower = 0;
                    autonomousStep += 1;
                }
                break;

            case 16:
                if (robot.canShoot()){
                    robot.setActivelyShooting(true);
                    startingTime = System.currentTimeMillis();
                    autonomousStep += 1.00;
                }
                break;
            case 17: //shoot part 2
                if (System.currentTimeMillis() - startingTime >= 1000){
                    robot.setActivelyShooting(false);
                    autonomousStep += 1.0;
                }
                break;
            case 18: //End of autonomous, wait for Teleop
                leftPower = 0;
                rightPower = 0;
                break;

        }
        robot.drivePercent(leftPower, rightPower);
    }

}
