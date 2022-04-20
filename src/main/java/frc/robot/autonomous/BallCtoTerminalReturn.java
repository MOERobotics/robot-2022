package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.command.PixyAutoTrack;
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
    double highPower = .95;
    double correction;
    boolean time = false;

    double distanceC = 47.9;
    double distanceTerminal = 218;
    double angleC = 84.74;
    double rampDownDist = 36;


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
    boolean targetFoundA = false;
    boolean targetFoundB = false;

    PixyAutoTrack pixyAutoTrack;



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
        targetFoundA = false;
        targetFoundB = false;

        pixyAutoTrack = new PixyAutoTrack(PIDDriveStraight);
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
        average += .5;

        double currentTurretPower = 0;

        if(robot.isTargetFound()){
            currentTurretPower = PIDTurret.calculate(average);
        }else{
            PIDTurret.reset();
        }

        if (autonomousStep <= 4 && !targetFoundA){
            robot.setShooterTargetRPM(2700);
            robot.setTurretPitchPosition(.272);
            if((!robot.isTargetFound()) && (System.currentTimeMillis() - startTime < 5000)) {
                currentTurretPower = .3;
            }
        }
        if ((autonomousStep>=7) && (autonomousStep < 8) && !targetFoundB){
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
            if (autonomousStep>4 &&targetFoundA) {
                robot.setShooterTargetRPM(robot.findShooterRPM());
                robot.setTurretPitchPosition(robot.findShooterPitch());

            }
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
                if (System.currentTimeMillis() - startTime >= 500){
                    autonomousStep += 1;
                    startingYaw = robot.getYaw();
                    startDistance = robot.getDriveDistanceInchesLeft();
                }
                break;
            case 1: //drive to ball C
                if (robot.isTargetFound()){
                    targetFoundA = true;
                }
                collectorPct = 1;
                correction = PIDDriveStraight.calculate(robot.getYaw() - startingYaw);

                leftpower = defaultPower + correction;
                rightpower = defaultPower - correction;

                if (robot.getDriveDistanceInchesLeft() - startDistance <= rampDownDist){
                    double rampUp = rampDown(defaultPower-.2, 0, startDistance, rampDownDist,
                            robot.getDriveDistanceInchesLeft(), rampDownDist);
                    leftpower = defaultPower - rampUp + correction;
                    rightpower = defaultPower - rampUp - correction;
                }
                if(robot.getDriveDistanceInchesLeft() - startDistance >= distanceC - rampDownDist){
                    double ramp = rampDown(defaultPower, .1, startDistance, rampDownDist,
                            robot.getDriveDistanceInchesLeft(), distanceC);
                    leftpower = ramp + correction;
                    rightpower = ramp - correction;
                }
                if(robot.getDriveDistanceInchesLeft() - startDistance >= distanceC){
                    autonomousStep += 1;

                    startTime = System.currentTimeMillis();
                }
                break;
            case 2: //stop
                if (robot.isTargetFound()){
                    targetFoundA = true;
                }
                leftpower = 0;
                rightpower = 0;
                startDistance = robot.getDriveDistanceInchesLeft();
                if (System.currentTimeMillis() - startTime >= 500){
                    robot.setPipeline(1);
                    autonomousStep += 1;
                }
                time = false;
                break;
            case 3: //create a target shooter value and see if shooter reaches it.
                if (robot.isTargetFound()){
                    targetFoundA = true;
                }
                if (robot.isTargetFound() && robot.canShoot() && (-5 < average) && (average< 5)){
                    robot.setActivelyShooting(true);
                    startTime = System.currentTimeMillis();
                    autonomousStep += 1;
                }
                break;
            case 4: //turn the shooter off
                System.out.print("TargetRPM:");
                System.out.print(robot.findShooterRPM());
                System.out.print(" Pitch:");
                System.out.print(robot.findShooterPitch());
                System.out.print(" What I have:");
                System.out.println(robot.getShooterRPMTop());
                if (System.currentTimeMillis() - startTime >= 1300){
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

                    pixyAutoTrack.resetCorrection();
                }
                break;
            case 7: //drive towards Ball Terminal
                if(robot.isTargetFound()){
                    targetFoundB = true;
                }

                double distanceTravelled = robot.getDriveDistanceInchesLeft() - startDistance;

                double startPixyDist = distanceTerminal - pixyAutoTrack.getPixyDistFar();
                double endPixyDist = distanceTerminal - pixyAutoTrack.getPixyDistNear();
                if(distanceTravelled >= startPixyDist && distanceTravelled <= endPixyDist){
                    pixyAutoTrack.updateReqCorrection(robot, defaultPower, startingYaw);
                }

                correction = pixyAutoTrack.getPIDCorrection(robot, startingYaw);


                leftpower = highPower + correction;
                rightpower = highPower - correction;

                if (robot.getDriveDistanceInchesLeft() - startDistance <= rampDownDist){
                    double rampUp = rampDown(highPower-.2, 0, startDistance, rampDownDist,
                            robot.getDriveDistanceInchesLeft(), rampDownDist);
                    leftpower = highPower - rampUp + correction;
                    rightpower = highPower - rampUp - correction;
                }


                if(robot.getDriveDistanceInchesLeft() - startDistance >= distanceTerminal - rampDownDist){
                    double ramp = rampDown(highPower, .2, startDistance, rampDownDist,
                            robot.getDriveDistanceInchesLeft(), distanceTerminal);
                    leftpower = ramp + correction;
                    rightpower = ramp - correction;
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
                if (System.currentTimeMillis() - startTime >= 1500){
                    startTime = System.currentTimeMillis();
                    autonomousStep += 1;
                }
                //TODO: at some point test how long we actually have to wait to collect the other ball
                break;
            case 9: //Drive back to Ball C
                correction = PIDDriveStraight.calculate(robot.getYaw() - (startingYaw+20));

                leftpower = -1*(highPower - correction);
                rightpower = -1*(highPower + correction);

                if (Math.abs(robot.getDriveDistanceInchesLeft() - startDistance) <= rampDownDist){
                    double rampUp = rampDown(highPower-.2, 0, startDistance, rampDownDist,
                            robot.getDriveDistanceInchesLeft(), rampDownDist);
                    leftpower = -highPower + rampUp + correction;
                    rightpower = -highPower + rampUp - correction;
                }

                if(Math.abs(robot.getDriveDistanceInchesLeft() - startDistance) >= distanceTerminal-130 - rampDownDist){
                    double ramp = rampDown(highPower, 0.2, startDistance, rampDownDist,

                            robot.getDriveDistanceInchesLeft(), distanceTerminal-130);
                    leftpower = -ramp + correction;
                    rightpower = -ramp - correction;
                }
                if(Math.abs(robot.getDriveDistanceInchesLeft() - startDistance) >= distanceTerminal-130){
                    leftpower = 0;
                    rightpower = 0;
                    autonomousStep += 1;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 10: //delay
                if (System.currentTimeMillis() - startTime >= 800){
                    autonomousStep += 1;
                }
                break;
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
                System.out.print(robot.getTurretPitchPosition());
                System.out.print("and my turret angle");
                System.out.println(robot.getAlternateTurretAngle());
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
