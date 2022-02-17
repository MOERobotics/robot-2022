package frc.robot.autonomous;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;

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
        // Turret Auto Track

        if(robot.isTargetFound()) {
            //take a look at averaging function
            //when we lose sight of target and then see it again.
            counter = counter%averageTurretXSize;
            averageX[counter] = robot.getTargetX();
            counter++;
        }
        average = 0;
        for(double i: averageX){
            average += i;
        }
        average /= averageTurretXSize;

        if (robot.isTargetFound()){
            currentTurretPower = turretPIDController.calculate(average);
        }else{
            turretPIDController.reset();
            currentTurretPower = 0;
        }

        // Turret AutoTrack

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
                    defaultPower = (rollout-currentDistInches+startInches)*defaultPower/rampDownDist;
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
                currentTurretPower = .05;
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
        robot.setTurretPowerPct(currentTurretPower);

    }

}
