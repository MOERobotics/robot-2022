package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.generic.GenericRobot;

public class TurretTracker {
    int averageTurretXSize = 2;
    double[] averageTurretX = new double [averageTurretXSize];
    double turretx;
    double turretv;
    int counter = 0;
    PIDController turretPIDController;

    public void turretInit(GenericRobot robot) {
        turretPIDController = new PIDController(robot.turretPIDgetP(), robot.turretPIDgetI(), robot.turretPIDgetD());
    }

    public void turretMove(GenericRobot robot) {
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
    }
}
