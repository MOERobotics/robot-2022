package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;

//Simple autonomous code for ball A, closest ball to the scoring table, and driving to the ball at terminal
public class BallAtoTerminal extends GenericAutonomous {
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

    double distanceA = 44.2;
    double distanceTerminal = 259.26;
    double angleA = 88.74;
    double rampDownDist = 10;

    PIDController PIDDriveStraight;
    PIDController PIDPivot;
    TurretTracker tracker = new TurretTracker();

    @Override
    public void autonomousInit(GenericRobot robot) {
        autonomousStep = 0;
        startingYaw = robot.getYaw();
        startTime = System.currentTimeMillis();
        shooterTargetRPM = 0;
        readyToShoot = false;
        PIDDriveStraight = new PIDController(robot.getPIDmaneuverP(), robot.getPIDmaneuverI(), robot.getPIDpivotD());
        PIDPivot = new PIDController(robot.getPIDpivotP(), robot.getPIDpivotI(), robot.getPIDpivotD());
        tracker.turretInit(robot);
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {
        tracker.turretUpdate(robot);

        if (autonomousStep >= 1){
            robot.getCargo();
            robot.shoot();
            robot.setTurretPitchPosition(.38);
        }
        switch(autonomousStep){
            case 0: //reset
                robot.lowerCollector();
                PIDDriveStraight.reset();
                PIDPivot.reset();
                PIDDriveStraight.enableContinuousInput(-180,180);
                PIDPivot.enableContinuousInput(-180,180);
                robot.resetEncoders();
                robot.resetAttitude();
                if (System.currentTimeMillis() - startTime > 100){
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
                    autonomousStep += 1;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 2: //stop
                leftpower = 0;
                rightpower = 0;
                if (System.currentTimeMillis() - startTime > 1000){
                    autonomousStep += 1;
                }
                break;
            case 3: //shoot the ball if target is found
                if (robot.canShoot()){
                    robot.setActivelyShooting(true);
                    startTime = System.currentTimeMillis();
                    autonomousStep += 1;
                }
                break;
            case 4: //turn shooter off
                if (System.currentTimeMillis()-startTime >= 1000){
                    robot.setActivelyShooting(false);
                    autonomousStep += 1;
                }
                break;
            case 5: //reset
                autonomousStep += 1;
                break;
            case 6: //turn to go to ball @ terminal
                correction = PIDPivot.calculate(-angleA + robot.getYaw() - startingYaw);
                leftpower = correction;
                rightpower = -correction;
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
                    startingYaw = -angleA;
                    startDistance = robot.getDriveDistanceInchesLeft();
                    startTime = System.currentTimeMillis();
                }
                break;

            case 7: //drive towards the ball
                //SKIPPED RIGHT NOW
                correction = PIDDriveStraight.calculate(robot.getYaw() - startingYaw);

                leftpower = defaultPower + correction;
                rightpower = defaultPower - correction;

                if(robot.getDriveDistanceInchesLeft() - startDistance >= distanceTerminal - rampDownDist){
                    double ramp = rampDown(defaultPower, .1, startDistance, rampDownDist,
                            robot.getDriveDistanceInchesLeft(), distanceTerminal);
                    leftpower = ramp;
                    rightpower = ramp;
                }
                if(robot.getDriveDistanceInchesLeft() - startDistance >= distanceTerminal) {
                    autonomousStep += 1;
                    leftpower = 0;
                    rightpower = 0;
                }
                break;
            case 8:
                leftpower = 0;
                rightpower = 0;
                break;
        }
        robot.drivePercent(leftpower, rightpower);
        tracker.turretMove(robot);

    }
}
