package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericAutonomous;
import frc.robot.generic.GenericRobot;

//Simple autonomous code for ball A, closest ball to the scoring table, and driving to the ball at terminal
public class SimpleATerminal extends GenericAutonomous {
    double startingYaw;

    int autonomousStep;

    double leftpower;
    double rightpower;
    double defaultPower = .25;
    double defaultTurnPower = .25;

    double correction;
    double startTime;

    double startDistance;

    PIDController PIDDriveStraight;

    //<Turret>
    int averageTurretXSize = 2;
    double[] averageTurretX = new double [averageTurretXSize];
    double turretx;
    double turrety;
    double turretarea;
    double turretv;
    int counter = 0;
    PIDController turretPIDController;
    //</Turret>


    @Override
    public void autonomousInit(GenericRobot robot) {
        autonomousStep = 0;
        startingYaw = robot.getYaw(); //might need to change to set degrees
        startTime = System.currentTimeMillis();
        PIDDriveStraight = new PIDController(robot.getPIDmaneuverP(), robot.getPIDmaneuverI(), robot.getPIDpivotD());

        turretPIDController = new PIDController(robot.turretPIDgetP(), robot.turretPIDgetI(), robot.turretPIDgetD());
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot) {
        //<Turret>
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
        NetworkTableEntry tv = table.getEntry("tv");

        turretx = tx.getDouble(0.0);
        turrety = ty.getDouble(0.0);
        turretarea = ta.getDouble(0.0);
        turretv = tv.getDouble(0.0);
        //</Turret>


        SmartDashboard.putNumber("Autonomous Step", autonomousStep);
        SmartDashboard.putNumber("Position", robot.getDriveDistanceInchesLeft());
        SmartDashboard.putNumber("Starting Yaw", startingYaw);
        SmartDashboard.putNumber("Current Yaw", robot.getYaw());

        SmartDashboard.putNumber("tx", turretx);
        SmartDashboard.putNumber("ty", turrety);
        SmartDashboard.putNumber("ta", turretarea);
        SmartDashboard.putNumber("tv", turretv);

        switch(autonomousStep){
            case 0: //reset
                PIDDriveStraight.reset();
                PIDDriveStraight.enableContinuousInput(-180,180);
                robot.resetEncoders();
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

                if(robot.getDriveDistanceInchesLeft() - startDistance >= 37){
                    autonomousStep += 1;
                    startTime = System.currentTimeMillis();
                } //has 3 inches of momentum with .25 power
                break;
            case 5: //stop
                leftpower = 0;
                rightpower = 0;
                if (System.currentTimeMillis() - startTime > 1000){
                    autonomousStep = 12;
                }
                //autonomousStep = 8;
                break;
            case 6: //collector to collect ball
            case 7: //collection part 2 not electric nor boogaloo
            case 8: //nother collection case
            case 9: //shoot the second ball for funsies
            case 10: //miss the target and become sadge
            case 11: //copium
            //will change these comments when they actually mean somthing
            case 12: //turn to go to ball @ terminal
                leftpower = defaultTurnPower;
                rightpower = -defaultTurnPower;
                //turning right

                if(robot.getYaw()- startingYaw > 87.74) {
                    startingYaw = robot.getYaw();
                    startDistance = robot.getDriveDistanceInchesLeft();
                    autonomousStep += 1;
                } //204.95
                break;
            case 13: //drive towards the ball
                correction = PIDDriveStraight.calculate(robot.getYaw() - startingYaw);

                leftpower = defaultPower + correction;
                rightpower = defaultPower - correction;

                if(robot.getDriveDistanceInchesLeft() - startDistance >= 259.26) {
                    autonomousStep += 1;
                    leftpower = 0;
                    rightpower = 0;
                } //might need to tune for momentum
                break;
            case 14:
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

        SmartDashboard.putNumber("Average", average);

        double currentTurretPower = 0;

        if(turretv !=0){
            currentTurretPower = turretPIDController.calculate(average);
        }else{
            turretPIDController.reset();
        }

        SmartDashboard.putNumber("currentTurretPower", currentTurretPower);
        robot.setTurretPowerPct(currentTurretPower);

    }
}
