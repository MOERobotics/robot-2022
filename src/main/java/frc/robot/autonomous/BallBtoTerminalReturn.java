package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;

//Simple autonomous code for ball B, ball between ball A and C
//Setup: put the robot down so that the ball at the terminal and the ball B are lined up straight.
public class BallBtoTerminalReturn extends GenericAutonomous {
    double startingYaw;
    double startDistance;
    double startTime;

    double leftpower;
    double rightpower;
    double defaultPower = .4;
    double correction;

    double distanceB = 61.5;
    double distanceTerminal = 160.6;
    double rampDownDist = 10;

    PIDController PIDTurret;

    int averageTurretXSize = 2;
    double[] averageTurretX = new double [averageTurretXSize];

    int counter = 0;

    PIDController PIDDriveStraight;

    @Override
    public void autonomousInit(GenericRobot robot) {
        autonomousStep = 0;
        startingYaw = robot.getYaw();
        PIDDriveStraight = new PIDController(robot.getPIDmaneuverP(), robot.getPIDmaneuverI(), robot.getPIDmaneuverD());
        startTime = System.currentTimeMillis();
        PIDTurret = new PIDController(robot.turretPIDgetP(), robot.turretPIDgetI(), robot.turretPIDgetD());
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {

        if (autonomousStep >= 1){
            robot.getCargo();
            robot.shoot();
            robot.setTurretPitchPosition(.38);
        }
        switch(autonomousStep){
            case 0: //reset
                robot.lowerCollector();
                PIDDriveStraight.reset();
                PIDDriveStraight.enableContinuousInput(-180,180);
                robot.resetEncoders();
                robot.resetAttitude();
                if (System.currentTimeMillis() - startTime > 100){
                    startDistance = robot.getDriveDistanceInchesLeft();
                    startingYaw = robot.getYaw();
                    autonomousStep += 1;
                }
                break;

            case 1: //drive to ball B
                correction = PIDDriveStraight.calculate(robot.getYaw() - startingYaw);

                leftpower = defaultPower + correction;
                rightpower = defaultPower - correction;

                if(robot.getDriveDistanceInchesLeft() - startDistance >= distanceB - rampDownDist){
                    double ramp = rampDown(defaultPower, .1, startDistance, rampDownDist,
                            robot.getDriveDistanceInchesLeft(), distanceB);
                    leftpower = ramp;
                    rightpower = ramp;
                }
                if(robot.getDriveDistanceInchesLeft() - startDistance >= distanceB){
                    autonomousStep += 1;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 2: //stop
                leftpower = 0;
                rightpower = 0;
                autonomousStep += 1;
                break;
            case 3:
                if (robot.canShoot()){
                    robot.setActivelyShooting(true);
                    startTime = System.currentTimeMillis();
                    autonomousStep += 1.0;
                }
                break;
            case 4: // part 2 not electric nor boogaloo
                if (System.currentTimeMillis() - startTime >= 1000){
                    robot.setActivelyShooting(false);
                    autonomousStep += 1;
                }
                break;

            case 5://reset
                if (System.currentTimeMillis() - startTime >= 1000){
                    PIDDriveStraight.reset();
                    PIDDriveStraight.enableContinuousInput(-180,180);
                    startDistance = robot.getDriveDistanceInchesLeft();
                    autonomousStep +=1;
                }
                break;
            case 6://drive to ball at terminal
                correction = PIDDriveStraight.calculate(robot.getYaw() - startingYaw);

                leftpower = defaultPower + correction;
                rightpower = defaultPower - correction;

                if(robot.getDriveDistanceInchesLeft() - startDistance >= distanceTerminal - rampDownDist){
                    double ramp = rampDown(defaultPower, .1, startDistance, rampDownDist, robot.getDriveDistanceInchesLeft(), distanceTerminal);
                    leftpower = ramp;
                    rightpower = ramp;
                }
                if(robot.getDriveDistanceInchesLeft() - startDistance >= distanceTerminal) {
                    autonomousStep += 1;
                    leftpower = 0;
                    rightpower = 0;
                }
                break;
            case 7:
                leftpower = 0;
                rightpower = 0;
                autonomousStep += 1.0;
                break;

            case 8:
                correction = PIDDriveStraight.calculate(robot.getYaw() - startingYaw);

                leftpower = -defaultPower + correction;
                rightpower = -defaultPower - correction;

                if(Math.abs(robot.getDriveDistanceInchesLeft() - startDistance) >= distanceTerminal - rampDownDist){
                    double ramp = rampDown(defaultPower, .1, startDistance, rampDownDist, robot.getDriveDistanceInchesLeft(), distanceTerminal);
                    leftpower = -ramp;
                    rightpower = -ramp;
                }
                if(Math.abs(robot.getDriveDistanceInchesLeft() - startDistance) >= distanceTerminal) {
                    autonomousStep += 1;
                    leftpower = 0;
                    rightpower = 0;
                }
                break;
            case 9:
                if (robot.canShoot()){
                    robot.setActivelyShooting(true);
                    startTime = System.currentTimeMillis();
                    autonomousStep += 1.00;
                }
                break;
            case 10:
                if (System.currentTimeMillis() - startTime >= 251){
                    robot.setActivelyShooting(false);
                    autonomousStep += 1;
                }
            case 11:
                leftpower = 0;
                rightpower = 0;
                break;
        }
        robot.drivePercent(leftpower, rightpower);

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
        if((!robot.isTargetFound()) && (System.currentTimeMillis() - startTime < 2000)) {
            currentTurretPower = .2;
        }
        robot.setTurretPowerPct(currentTurretPower);

    }
}
