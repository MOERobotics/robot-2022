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
    double distanceTerminal = 159.6;
    double rampDownDist = 10;

    PIDController PIDTurret;

    int averageTurretXSize = 2;
    double[] averageTurretX = new double [averageTurretXSize];

    int counter = 0;

    PIDController PIDDriveStraight;

    boolean targetFoundA = false;
    boolean autoTarg = false;

    @Override
    public void autonomousInit(GenericRobot robot) {
        autonomousStep = 0;
        startingYaw = robot.getYaw();
        PIDDriveStraight = new PIDController(robot.getPIDmaneuverP(), robot.getPIDmaneuverI(), robot.getPIDmaneuverD());
        startTime = System.currentTimeMillis();
        PIDTurret = new PIDController(robot.turretPIDgetP(), robot.turretPIDgetI(), robot.turretPIDgetD());
        robot.setPipeline(0);
        autoTarg = false;

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
        average += 2;

        double currentTurretPower = 0;

        if(robot.isTargetFound()){
            currentTurretPower = PIDTurret.calculate(average);
        }else{
            PIDTurret.reset();
        }
        if((!robot.isTargetFound()) && !targetFoundA) {
            currentTurretPower = .45;
        }

        robot.setTurretPowerPct(currentTurretPower);
        //////////AUTO TRACK STUFF


        if (autonomousStep >= 1){
            robot.getCargo();
            robot.shoot();
            if (autonomousStep >= 5) {
                robot.setShooterTargetRPM(robot.findShooterRPM());
                robot.setTurretPitchPosition(robot.findShooterPitch());
            }
            else{
                if (!autoTarg) {
                    robot.setShooterTargetRPM(2520);
                    robot.setTurretPitchPosition(.307);
                }
                else{
                    robot.setShooterTargetRPM(robot.findShooterRPM());
                    robot.setTurretPitchPosition(robot.findShooterPitch());
                }
            }
        }

        else{
            robot.setCollectorIntakePercentage(0);
            robot.setTurretPowerPct(0);
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
                if (robot.isTargetFound()){
                    targetFoundA = false;
                }
                correction = PIDDriveStraight.calculate(robot.getYaw() - startingYaw);

                leftpower = defaultPower + correction;
                rightpower = defaultPower - correction;

                if(robot.getDriveDistanceInchesLeft() - startDistance >= distanceB - rampDownDist){
                    double ramp = rampDown(defaultPower, .1, startDistance, rampDownDist,
                            robot.getDriveDistanceInchesLeft(), distanceB);
                    leftpower = ramp + correction;
                    rightpower = ramp - correction;
                }
                if(robot.getDriveDistanceInchesLeft() - startDistance >= distanceB){
                    autonomousStep += 1;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 2: //stop
                if (robot.isTargetFound()){
                    targetFoundA = false;
                }
                leftpower = 0;
                rightpower = 0;

                if (System.currentTimeMillis() - startTime >= 250){

                    robot.setPipeline(1);
                    autoTarg = true;
                    if (System.currentTimeMillis() - startTime >= 500) {
                        autonomousStep += 1;
                    }
                }
                break;
            case 3:
                if (robot.isTargetFound()){
                    targetFoundA = false;
                }
                if (robot.isTargetFound() && robot.canShoot() && (-5 < average) && (average < 5)){
                    robot.setActivelyShooting(true);
                    startTime = System.currentTimeMillis();
                    autonomousStep += 1.0;
                }
                break;
            case 4: // part 2 not electric nor boogaloo
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
            case 6://drive to ball at terminal
                correction = PIDDriveStraight.calculate(robot.getYaw() - startingYaw);

                leftpower = defaultPower + correction;
                rightpower = defaultPower - correction;



                if(robot.getDriveDistanceInchesLeft() - startDistance >= distanceTerminal - rampDownDist){
                    double ramp = rampDown(defaultPower, .1, startDistance, rampDownDist, robot.getDriveDistanceInchesLeft(), distanceTerminal);
                    leftpower = ramp + correction;
                    rightpower = ramp - correction;
                }
                if(robot.getDriveDistanceInchesLeft() - startDistance >= distanceTerminal) {
                    autonomousStep += 1;
                    leftpower = 0;
                    rightpower = 0;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 7:
                leftpower = 0;
                rightpower = 0;
                startDistance = robot.getDriveDistanceInchesLeft();
                if (System.currentTimeMillis() -startTime >= 2000){
                    autonomousStep += 1;
                }
                //TODO:check timing
                break;
            case 8:
                correction = PIDDriveStraight.calculate(robot.getYaw() - startingYaw);

                leftpower = -defaultPower + correction;
                rightpower = -defaultPower - correction;

                if(Math.abs(robot.getDriveDistanceInchesLeft() - startDistance) >= distanceTerminal - rampDownDist){
                    double ramp = rampDown(defaultPower, .1, startDistance, rampDownDist, robot.getDriveDistanceInchesLeft(), distanceTerminal);
                    leftpower = -ramp + correction;
                    rightpower = -ramp - correction;
                }
                if(Math.abs(robot.getDriveDistanceInchesLeft() - startDistance) >= distanceTerminal - 60) {
                    autonomousStep += 1;
                    leftpower = 0;
                    rightpower = 0;
                }
                break;
            case 9:
                if (robot.isTargetFound() && robot.canShoot()){
                    robot.setActivelyShooting(true);
                    startTime = System.currentTimeMillis();
                    autonomousStep += 1.00;
                }
                break;
            case 10:
                if (System.currentTimeMillis() - startTime >= 2000){
                    robot.setActivelyShooting(false);
                    autonomousStep += 1;
                }
            case 11:
                leftpower = 0;
                rightpower = 0;
                break;
        }
        robot.drivePercent(leftpower, rightpower);

    }
}
