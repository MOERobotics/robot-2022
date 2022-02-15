package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;


public abstract class GenericAutonomous {

    public int autonomousStep;

    public void autonomousInit(GenericRobot robot) {
        //System.out.println("I don't have autonomousInit in my autonomous program :'(");
    }

    public void autonomousPeriodic(GenericRobot robot) {
        //System.out.println("I don't have autonomousPeriodic in my autonomous program :'(");
    }
}