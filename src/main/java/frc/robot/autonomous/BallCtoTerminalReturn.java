package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.generic.GenericRobot;

//Simple autonomous code for ball C, closest ball to the hangar, and driving to the ball at terminal
//Setup: Line the robot straight between ball C and the center point of the hub
public class BallCtoTerminalReturn extends GenericAutonomous {
    double startingYaw;
    double startDistance;
    double startTime;

    double leftpower;
    double rightpower;
    double defaultPower = .75;
    double correction;
    boolean time = false;

    double distanceC = 47.9;
    double distanceTerminal = 223;
    double angleC = 84.74;
    double rampDownDist = 10;

    PIDController PIDDriveStraight;
    PIDController PIDTurret;
    PIDController PIDPivot;

    int averageTurretXSize = 2;
    double[] averageTurretX = new double [averageTurretXSize];
    int counter = 0;

    double shooterTargetRPM;
    boolean readyToShoot;

    double indexerPct;
    double collectorPct;



    @Override
    public void autonomousInit(GenericRobot robot) {
        autonomousStep = 0;
        startingYaw = robot.getYaw();
        startTime = System.currentTimeMillis();
        shooterTargetRPM = 0;
        indexerPct = 0;
        collectorPct = 0;
        readyToShoot = false;
        PIDDriveStraight = new PIDController(robot.getPIDmaneuverP(), robot.getPIDmaneuverI(), robot.getPIDmaneuverD());
        PIDPivot = new PIDController(robot.getPIDpivotP(), robot.getPIDpivotI(), robot.getPIDpivotD());
        PIDTurret = new PIDController(robot.turretPIDgetP(), robot.turretPIDgetI(), robot.turretPIDgetD());
        robot.setPipeline(0);
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {

        if(robot.isTargetFound()) {
            averageTurretX[counter % averageTurretXSize] = robot.getTargetX();
            counter++;
        }

        double average = 0;
        for(double i: averageTurretX){
            average += i;
        }
        average /= averageTurretXSize;

        double currentTurretPower = 0;

        if(robot.isTargetFound()){
            currentTurretPower = PIDTurret.calculate(average);
        }else{
            PIDTurret.reset();
        }

        if (autonomousStep < 4){
            if((!robot.isTargetFound()) && (System.currentTimeMillis() - startTime < 5000)) {
                currentTurretPower = .3;
            }
        }
        if ((autonomousStep>=4) && (autonomousStep < 8)){
            if((!robot.isTargetFound()) && (System.currentTimeMillis() - startTime < 5000)) {
                currentTurretPower = -.2;
            }
        }
        if (autonomousStep <= 13){
            robot.setTurretPowerPct(currentTurretPower);
        }

        if (autonomousStep >= 1 && autonomousStep <= 13){
            robot.getCargo();
            robot.shoot();
            robot.setShooterTargetRPM(robot.findShooterRPM());
            robot.setTurretPitchPosition(robot.findShooterPitch());
        }
        else{
            robot.setCollectorIntakePercentage(0);
            robot.setTurretPowerPct(0);
        }
        switch(autonomousStep){
            case 0: //reset
                robot.lowerCollector();
                PIDPivot.reset();
                PIDPivot.enableContinuousInput(-180,180);
                PIDDriveStraight.reset();
                PIDDriveStraight.enableContinuousInput(-180,180);
                robot.resetEncoders();
                robot.resetAttitude();
                if (System.currentTimeMillis() - startTime >= 1000){
                    autonomousStep += 1;
                    startingYaw = robot.getYaw();
                    startDistance = robot.getDriveDistanceInchesLeft();
                }
                break;
            case 1: //drive to ball C
                collectorPct = 1;
                correction = PIDDriveStraight.calculate(robot.getYaw() - startingYaw);

                leftpower = defaultPower + correction;
                rightpower = defaultPower - correction;

                if(robot.getDriveDistanceInchesLeft() - startDistance >= distanceC - rampDownDist){
                    double ramp = rampDown(defaultPower, .1, startDistance, rampDownDist,
                            robot.getDriveDistanceInchesLeft(), distanceC);
                    leftpower = ramp;
                    rightpower = ramp;
                }
                if(robot.getDriveDistanceInchesLeft() - startDistance >= distanceC){
                    autonomousStep += 1;
                    robot.setPipeline(1);
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
            case 3: //create a target shooter value and see if shooter reaches it.
                if (robot.isTargetFound() && robot.canShoot() && (-5 < average) && (average< 5)){
                    robot.setActivelyShooting(true);
                    startTime = System.currentTimeMillis();
                    autonomousStep += 1;
                }
                break;
            case 4: //turn the shooter off
                if (System.currentTimeMillis() - startTime >= 2000){
                    robot.setActivelyShooting(false);
                    autonomousStep += 1;
                }
                break;

            case 5://reset
                PIDDriveStraight.reset();
                PIDDriveStraight.enableContinuousInput(-180,180);
                startDistance = robot.getDriveDistanceInchesLeft();
                autonomousStep +=1;
                break;
            case 6: //turn to go to Ball Terminal
                correction = PIDPivot.calculate(angleC + robot.getYaw() - startingYaw );
                leftpower = correction;
                rightpower = -correction;
                //turning left
                if (Math.abs(Math.abs(robot.getYaw() - startingYaw)-angleC) <= 1.5){
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
                    startingYaw = -angleC;
                    startDistance = robot.getDriveDistanceInchesLeft();
                    startTime = System.currentTimeMillis();
                }
                break;
            case 7: //drive towards Ball Terminal

                correction = PIDDriveStraight.calculate(robot.getYaw() - startingYaw);

                leftpower = defaultPower + correction;
                rightpower = defaultPower - correction;

                if(robot.getDriveDistanceInchesLeft() - startDistance >= distanceTerminal - rampDownDist){
                    double ramp = rampDown(defaultPower, .1, startDistance, 10,
                            robot.getDriveDistanceInchesLeft(), distanceTerminal);
                    leftpower = ramp;
                    rightpower = ramp;
                }
                if(robot.getDriveDistanceInchesLeft() - startDistance >= distanceTerminal){
                    autonomousStep += 1;
                    startDistance = robot.getDriveDistanceInchesLeft();
                    startTime = System.currentTimeMillis();
                }
                break;
            case 8: //collect Ball Terminal & stop to collect another ball from player
                leftpower = 0;
                rightpower = 0;
                autonomousStep += 1;
                //TODO: at some point test how long we actually have to wait to collect the other ball
                break;
            case 9: //Drive back to Ball C
                correction = PIDDriveStraight.calculate(robot.getYaw() - startingYaw);

                leftpower = -1*(defaultPower - correction);
                rightpower = -1*(defaultPower + correction);

                if (Math.abs(robot.getDriveDistanceInchesLeft() - startDistance) <= rampDownDist){
                    double rampUp = rampDown(defaultPower, 0.1, startDistance, rampDownDist,
                            robot.getDriveDistanceInchesLeft(), rampDownDist);
                    leftpower = defaultPower - rampUp;
                    rightpower = defaultPower - rampUp;
                }



                if(Math.abs(robot.getDriveDistanceInchesLeft() - startDistance) >= distanceTerminal-50 - rampDownDist){
                    double ramp = rampDown(defaultPower, 0, startDistance, 10,

                            robot.getDriveDistanceInchesLeft(), distanceTerminal-50);
                    leftpower = -ramp;
                    rightpower = -ramp;
                }
                if(Math.abs(robot.getDriveDistanceInchesLeft() - startDistance) >= distanceTerminal-98){
                    leftpower = 0;
                    rightpower = 0;
                    autonomousStep += 1;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 10: //delay
                if (System.currentTimeMillis() - startTime >= 400){
                    autonomousStep += 1;
                }
            case 11: //shoot it :)
                if (robot.canShoot()){
                    robot.setActivelyShooting(true);
                    startTime = System.currentTimeMillis();
                    autonomousStep += 1.00;
                }
                break;
            case 12: //shoot part 2
                System.out.print("I see a target, this is my rpm: ");
                System.out.print(robot.getShooterRPMTop());
                System.out.print(" and my pitch: ");
                System.out.println(robot.getTurretPitchPosition());
                if (System.currentTimeMillis() - startTime >= 2000){
                    robot.setActivelyShooting(false);
                    autonomousStep += 1.0;
                }
                break;
            case 13: //End of autonomous, wait for Teleop
                leftpower = 0;
                rightpower = 0;
                break;
        }
        robot.drivePercent(leftpower, rightpower);
    }
}
