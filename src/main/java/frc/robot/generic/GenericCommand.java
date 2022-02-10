package frc.robot.generic;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class GenericCommand {

    public void begin(GenericRobot robot) {
        System.out.println("I don't define begin() steps in my command :'(");
    }

    public void step(GenericRobot robot) {
        System.out.println("I don't define begin() steps in my command :'(");
    }

    public boolean controlLock;

    public boolean enable;


}
