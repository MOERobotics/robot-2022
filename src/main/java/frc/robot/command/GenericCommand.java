package frc.robot.command;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;

public abstract class GenericCommand {
    public int commandStep = -1;

    public void begin(GenericRobot robot) {
        System.out.println("I don't define begin() steps in my command :'(");
    }

    public void step(GenericRobot robot) {
        System.out.println("I don't define begin() steps in my command :'(");
    }

    public boolean controlLock;

    public boolean enable;

    public double rampDown(double startPower, double endPower, double startDistance, double rolloutDistance, double currentDist, double endDist){
        double power = (endDist-Math.abs(currentDist-startDistance))*(startPower-endPower)/rolloutDistance+endPower;
        return power;
    }


}
