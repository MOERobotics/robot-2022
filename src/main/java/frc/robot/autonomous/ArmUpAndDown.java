package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.GenericAutonomous;
import frc.robot.command.GenericCommand;
import frc.robot.generic.GenericRobot;

public class ArmUpAndDown extends GenericAutonomous {


    /////^^^^^^^^^^^Stuff for tapeAlign/


    //////////////Now the real stuff
    double startTime;
    double escapeHeight = 5;///TODO:what is this??
    boolean firstTime = true;
    int countLeft = 0;
    int countRight = 0;
    double leftArmPower = 0;
    double rightArmPower = 0;
    double defaultClimbPowerUp = .3;
    double defaultClimbPowerDown = -.3;
    boolean leftArrived = false;
    boolean rightArrived = false;
    double startHeightLeft = 0;
    double startHeightRight = 0;
    double turretPower = 0;
    double level = 7;
    double leveltol = 2;
    double topHeight = 26;
    double topExtend = 31;
    boolean swingTime = false;
    double origPitch;
    double criticalHeight = 21;

    PIDController turretPIDController;

    @Override
    public void autonomousInit(GenericRobot robot) {
        autonomousStep = 0;
        rightArrived = false;
        leftArrived = false;
    }

    @Override
    public void autonomousPeriodic(GenericRobot robot){

        SmartDashboard.putNumber("RightHeight", robot.armHeightRight()-startHeightRight);
        SmartDashboard.putNumber("LeftHeight", robot.armHeightLeft()-startHeightLeft);

        switch (autonomousStep){

            case 0:
                robot.turnOnPTO();
                if (!leftArrived){
                    startTime = System.currentTimeMillis();
                    leftArrived = true;
                }
                if (System.currentTimeMillis() - startTime >= 1000){
                    leftArrived = false;
                    autonomousStep += 1;
                }
                break;
            case 1:
                if (!robot.getClimbSensorRight()){
                    startHeightRight = robot.armHeightRight();
                    rightArrived = true;
                }
                if (!robot.getClimbSensorLeft()){
                    startHeightLeft = robot.armHeightLeft();
                    leftArrived = true;
                }
                autonomousStep += 1;
                break;
            case 2:
                if (!rightArrived){
                    rightArmPower = defaultClimbPowerDown;
                }
                if (!leftArrived){
                    leftArmPower = defaultClimbPowerDown;
                }
                if (!robot.getClimbSensorLeft()){
                    leftArrived = true;
                    leftArmPower = 0;
                    startHeightLeft = robot.armHeightLeft();
                }
                if(!robot.getClimbSensorRight()){
                    rightArmPower = 0;
                    rightArrived = true;
                    startHeightRight = robot.armHeightRight();
                }
                if (leftArrived && rightArrived){
                    leftArrived = false;
                    rightArrived = false;
                    autonomousStep += 1;
                }
                break;
            case 3:

                if (robot.armHeightLeft() - startHeightLeft >= 25){
                    leftArmPower = 0;
                    leftArrived = true;
                }
                else{
                    leftArmPower = defaultClimbPowerUp;
                }

                if (robot.armHeightRight() - startHeightRight >= 25){
                    rightArmPower = 0;
                    rightArrived = true;
                }
                else{
                    rightArmPower = defaultClimbPowerUp;
                }


                if (leftArrived && rightArrived){
                    rightArmPower = 0;
                    leftArmPower = 0;
                    autonomousStep += 1;
                }
                break;
            case 4:
                leftArmPower = 0;
                rightArmPower = 0;
                break;

        }
        robot.armPower(leftArmPower, rightArmPower);
    }

}
