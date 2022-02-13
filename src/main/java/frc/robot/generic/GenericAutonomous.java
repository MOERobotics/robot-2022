package frc.robot.generic;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;




public abstract class GenericAutonomous {


    public void autonomousInit(GenericRobot robot) {
        System.out.println("I don't have autonomousInit in my autonomous program :'(");
    }

    public void autonomousPeriodic(GenericRobot robot) {
        System.out.println("I don't have autonomousPeriodic in my autonomous program :'(");
    }
}