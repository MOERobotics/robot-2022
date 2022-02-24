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
    double defaultPower = .4;
    double defaultTurnPower = .4;
    double correction;
    boolean time = false;

    double distanceC = 47.9;
    double distanceTerminal = 251;
    double angleC = 82.74;
    double rampDownDist = 10;

    PIDController PIDDriveStraight;
    PIDController PIDTurret;
    PIDController PIDPivot;

    int averageTurretXSize = 2;
    double[] averageTurretX = new double [averageTurretXSize];
    double turretx;
    double turrety;
    double turretarea;
    double turretv;
    int counter = 0;

    //TurretTracker tracker = new TurretTracker();


    boolean initialTurretSpin = true;
    @Override
    public void autonomousInit(GenericRobot robot) {
        autonomousStep = 0;
        startingYaw = robot.getYaw();
        startTime = System.currentTimeMillis();

        PIDDriveStraight = new PIDController(robot.getPIDmaneuverP(), robot.getPIDmaneuverI(), robot.getPIDmaneuverD());
        PIDPivot = new PIDController(robot.getPIDpivotP(), robot.getPIDpivotI(), robot.getPIDpivotD());
        PIDTurret = new PIDController(robot.turretPIDgetP(), robot.turretPIDgetI(), robot.turretPIDgetD());

        //tracker.turretInit(robot);

    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
        NetworkTableEntry tv = table.getEntry("tv");

        turretx = tx.getDouble(0.0);
        turrety = ty.getDouble(0.0);
        turretarea = ta.getDouble(0.0);
        turretv = tv.getDouble(0.0);
        //tracker.turretUpdate(robot);





        switch(autonomousStep){
            case 0: //reset
                robot.lowerCollector();
                PIDPivot.reset();
                PIDPivot.enableContinuousInput(-180,180);
                PIDDriveStraight.reset();
                PIDDriveStraight.enableContinuousInput(-180,180);
                robot.resetEncoders();
                robot.resetAttitude();
                if (System.currentTimeMillis() - startTime > 100){
                    autonomousStep = 4;
                    startingYaw = robot.getYaw();
                    startDistance = robot.getDriveDistanceInchesLeft();
                }
                break;
            case 1: //shoot the ball
            case 2: //shoot the ball part 2 electric boogaloo
            case 3: //shoot the ball part 3 maybe
            case 4: //drive to ball A
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
                    startTime = System.currentTimeMillis();
                }
                break;
            case 5: //stop
                leftpower = 0;
                rightpower = 0;
                startDistance = robot.getDriveDistanceInchesLeft();
                autonomousStep = 12;
                time = false;
                break;
            case 6: //turret turn

            case 7: //collection part 2 not electric nor boogaloo
            case 8: //another collection case
            case 9: //shoot the second ball for funsies
            case 10: //miss the target and become sadge
            case 11: //copium
                //will change these comments when they actually mean something
            case 12://reset
                PIDDriveStraight.reset();
                PIDDriveStraight.enableContinuousInput(-180,180);
                startDistance = robot.getDriveDistanceInchesLeft();
                autonomousStep +=1;
                break;
            case 13: //turn to go to ball @ terminal
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
            case 14: //drive towards the ball
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
            case 15:
                leftpower = 0;
                rightpower = 0;
                autonomousStep += 1;
                break;
            case 16:
                correction = PIDDriveStraight.calculate(robot.getYaw() - startingYaw);

                leftpower = -1*(defaultPower - correction);
                rightpower = -1*(defaultPower + correction);

                if(Math.abs(robot.getDriveDistanceInchesLeft() - startDistance) >= distanceTerminal - rampDownDist){
                    double ramp = rampDown(defaultPower, 0, startDistance, 10,
                            robot.getDriveDistanceInchesLeft(), distanceTerminal);
                    leftpower = -ramp;
                    rightpower = -ramp;
                }
                if(Math.abs(robot.getDriveDistanceInchesLeft() - startDistance) >= distanceTerminal){
                    leftpower = 0;
                    rightpower = 0;
                    autonomousStep += 1;
                }
                break;
            case 17:
                leftpower = 0;
                rightpower = 0;
                break;
        }
        robot.drivePercent(leftpower, rightpower);
        //If turret works set value of averageTurretX[] to turretx


        if(turretv !=0 ) {
            averageTurretX[counter % averageTurretXSize] = turretx;
            counter++;
        }

        double average = 0;
        for(double i: averageTurretX){
            average += i;
        }
        average /= averageTurretXSize;

        double currentTurretPower = 0;

        if(turretv !=0){
            currentTurretPower = PIDTurret.calculate(average);
        }else{
            PIDTurret.reset();
        }
        if((turretv == 0) && (System.currentTimeMillis() - startTime < 2000)) {
            currentTurretPower = .2;
        }
        robot.setTurretPowerPct(currentTurretPower);

        //tracker.turretMove(robot);
    }
}
