package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;


public abstract class GenericAutonomous {

    public int autonomousStep = 0;

    public void autonomousInit(GenericRobot robot) {
        //System.out.println("I don't have autonomousInit in my autonomous program :'(");
    }

    public void autonomousPeriodic(GenericRobot robot) {
        //System.out.println("I don't have autonomousPeriodic in my autonomous program :'(");
    }


    public double rampDown(double startPower, double endPower, double startDistance, double rolloutDistance, double currentDist, double endDist){
        double power = (endDist-currentDist+startDistance)*(startPower-endPower)/rolloutDistance+endPower;
        return power;
    }
}