package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;

//Simple autonomous code for ball C, closest ball to the hangar, and driving to the ball at terminal
//Setup: 33.21 degrees from the white dividing line
public class BallCtoTerminal extends GenericAutonomous {
    double startingYaw;
    double startDistance;
    double startTime;

    double leftpower;
    double rightpower;
    double defaultPower = .4;
    double defaultTurnPower = .4;
    double correction;

    double distanceC = 47.9;
    double distanceTerminal = 251;
    double angleC = 83.74; //og = 84.74
    double rampDownDist = 10;

    PIDController PIDDriveStraight;

    int averageTurretXSize = 2;
    double[] averageTurretX = new double [averageTurretXSize];
    double turretx;
    double turrety;
    double turretarea;
    double turretv;
    int counter = 0;
    PIDController turretPIDController;
    //TurretTracker tracker = new TurretTracker();

    @Override
    public void autonomousInit(GenericRobot robot) {
        autonomousStep = 0;
        startingYaw = robot.getYaw();
        startTime = System.currentTimeMillis();
        PIDDriveStraight = new PIDController(robot.getPIDmaneuverP(), robot.getPIDmaneuverI(), robot.getPIDmaneuverD());

        turretPIDController = new PIDController(robot.turretPIDgetP(), robot.turretPIDgetI(), robot.turretPIDgetD());
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
                break;
            case 6: //collector to collect ball
            case 7: //collection part 2 not electric nor boogaloo
            case 8: //another collection case
            case 9: //shoot the second ball for funsies
            case 10: //miss the target and become sadge
            case 11: //copium
            //will change these comments when they actually mean something
            case 12://reset
                if (System.currentTimeMillis() - startTime >= 1000){
                    PIDDriveStraight.reset();
                    PIDDriveStraight.enableContinuousInput(-180,180);
                    startDistance = robot.getDriveDistanceInchesLeft();
                    autonomousStep +=1;
                }
                break;
            case 13: //turn to go to ball @ terminal
                leftpower = -defaultTurnPower;
                rightpower = defaultTurnPower;
                //turning left

                if(robot.getYaw() - startingYaw < -angleC) {
                    startingYaw = startingYaw - angleC;
                    startDistance = robot.getDriveDistanceInchesLeft();
                    PIDDriveStraight.reset();
                    autonomousStep += 1;
                }
                break;
            case 14: //drive towards the ball
                correction = PIDDriveStraight.calculate(robot.getYaw() - startingYaw);

                leftpower = defaultPower + correction;
                rightpower = defaultPower - correction;

                if(robot.getDriveDistanceInchesLeft() - startDistance >= distanceTerminal - rampDownDist){
                    double ramp = rampDown(defaultPower, 0, startDistance, 10,
                            robot.getDriveDistanceInchesLeft(), distanceTerminal);
                    leftpower = ramp;
                    rightpower = ramp;
                }
                if(robot.getDriveDistanceInchesLeft() - startDistance >= distanceTerminal){
                    autonomousStep += 1;
                    startTime = System.currentTimeMillis();
                }
                break;
            case 15:
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
            currentTurretPower = turretPIDController.calculate(average);
        }else{
            turretPIDController.reset();
        }

        robot.setTurretPowerPct(currentTurretPower);
        //tracker.turretMove(robot);
    }
}
