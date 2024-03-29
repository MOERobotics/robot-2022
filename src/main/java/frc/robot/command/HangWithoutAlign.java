package frc.robot.command;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;

public class HangWithoutAlign extends GenericCommand{


    double startAngle;

    double leftPower;
    double rightPower;
    double defaultPower = .4;

    double startDistance;
    double differenceDistance;

    double sensorDist = 21.0;
    double Tapetheta = 0;

    double correction;
    double currentYaw;

    long startingTime = 0;

    boolean leftSensor = false;
    boolean rightSensor = false;

    double lTraveled;

    double fwd = 49.5;
    PIDController PIDSteering;
    boolean tapeAlign;

    /////^^^^^^^^^^^Stuff for tapeAlign/


    //////////////Now the real stuff
    double escapeHeight = 10;///TODO:what is this??
    boolean firstTime = true;
    int countLeft = 0;
    int countRight = 0;
    double leftArmPower = 0;
    double rightArmPower = 0;
    double defaultClimbPowerUp = .75;
    double defaultClimbPowerDown = -.75;
    boolean leftArrived = false;
    boolean rightArrived = false;
    double startHeightLeft = 0;
    double startHeightRight = 0;
    double turretPower = 0;
    double level = 7;
    double leveltol = 2;

    PIDController turretPIDController;


    public void begin(GenericRobot robot){
        startingTime = System.currentTimeMillis();
        commandStep = -1;
        leftSensor = false;
        rightSensor = false;
        leftArmPower = 0;
        rightArmPower = 0;
        leftPower = 0;
        rightPower = 0;
        lTraveled = 0;
        fwd = 49.5;
        PIDSteering = new PIDController(robot.getPIDmaneuverP(), robot.getPIDmaneuverI(), robot.getPIDmaneuverD());
        turretPIDController = new PIDController(robot.turretPIDgetP(), robot.turretPIDgetI(), robot.turretPIDgetD());
        tapeAlign = true;
        firstTime = true;
    }

    public void step(GenericRobot robot){

        SmartDashboard.putNumber("countLeft", countLeft);
        SmartDashboard.putNumber("countRight", countRight);
        SmartDashboard.putNumber("startDistance", startDistance);


    switch (commandStep){
            case -1:
                commandStep += 1;
                break;
            case 0:///reset and enable PTO
                //reset encoders
                robot.turnOnPTO();
                robot.resetEncoders();
                leftArmPower = 0;
                rightArmPower = 0;
                countLeft = 0;
                countRight = 0;
                if (System.currentTimeMillis() - startingTime >= 5000){
                    SmartDashboard.putNumber("driveOutputCurrent", robot.getDriveCurrent());
                    commandStep = 2; ///TODO: fix numbering
                }

                break;
            /*case 1:  ///////////unlock rotation piston to send arms forward
                robot.setArmsForward(); //TODO: skip step
                commandStep += 1;
                break;*/
            case 2: //////raise climber arms (skip 10 steps after in case we need to scoot/scoot

                if (!robot.getClimbSensorLeft() && countLeft == 0){
                    countLeft = 1;
                }

                if (robot.getClimbSensorLeft() && countLeft == 1){
                    leftArmPower = 0;
                    leftArrived = true;
                }
                else{
                    leftArmPower = defaultClimbPowerUp;
                }


                if (!robot.getClimbSensorRight() && countRight == 0){
                    countRight = 1;
                }

                if (robot.getClimbSensorRight() && countRight == 1){
                    rightArmPower = 0;
                    rightArrived = true;
                }
                else{
                    rightArmPower = defaultClimbPowerUp;
                }

                if (leftArrived && rightArrived){
                    leftArrived = false;
                    rightArrived = false;
                    countLeft = 0;
                    countRight = 0;
                    leftArmPower = 0;
                    rightArmPower = 0;
                    startingTime = System.currentTimeMillis();
                    commandStep += 1;

                }

                break;
            case 3:  ///////////unlock rotation piston to send arms back
                robot.setArmsBackward();
                if (System.currentTimeMillis() - startingTime >= 1000) {
                    commandStep = 11;
                }
                break;
            /*case 2: //////disable PTO
                robot.turnOffPTO();
                commandStep += 1;
                break;
            case 3: //////reset
                startDistance = robot.getDriveDistanceInchesLeft();
                commandStep += 1;
                break;
            case 4: //go back 8 in
                leftArmPower = -.1; //see if we should change to drive stuff
                rightArmPower = -.1;
                if (Math.abs(robot.getDriveDistanceInchesLeft() - startDistance) >= 8){
                    leftArmPower = 0;
                    rightArmPower = 0;
                    commandStep += 1;
                }
            case 5: //////enable PTO
                robot.turnOnPTO();
                commandStep = 11;
                break;*/
            case 11: ////////lower climber arms

                if (!robot.getClimbSensorLeft() && countLeft == 0){
                    countLeft = 1;
                }

                if (robot.getClimbSensorLeft() && countLeft == 1){
                    leftArmPower = 0;
                    leftArrived = true;
                }
                else{
                    leftArmPower = defaultClimbPowerDown;
                }


                if (!robot.getClimbSensorRight() && countRight == 0){
                    countRight = 1;
                }

                if (robot.getClimbSensorRight() && countRight == 1){
                    rightArmPower = 0;
                    rightArrived = true;
                }
                else{
                    rightArmPower = defaultClimbPowerDown;
                }
                if (robot.armInContact() && leftArrived && rightArrived){
                    countRight = 0;
                    countLeft = 0;
                    leftArmPower = 0;
                    rightArmPower = 0;
                    //TODO: change
                    commandStep += 1;
                    leftArrived = false;
                    rightArrived = false;
                    startHeightLeft = robot.armHeightLeft();
                    startHeightRight = robot.armHeightRight();
                    startingTime = System.currentTimeMillis();

                }
                break;

            case 12:  /////////////raise arms slightly
                if (Math.abs(robot.armHeightLeft()-startHeightLeft) >= escapeHeight){
                    leftArmPower = 0;
                    leftArrived = true;
                }
                else{
                    leftArmPower = defaultClimbPowerUp;
                }
                if (Math.abs(robot.armHeightRight()-startHeightRight) >= escapeHeight){
                    rightArmPower = 0;
                    rightArrived = true;
                }
                else{
                    rightArmPower = defaultClimbPowerUp;
                }
                if (rightArrived && leftArrived){
                    rightArmPower = 0;
                    leftArmPower = 0;
                    rightArrived = false;
                    leftArrived = false;
                    commandStep += 1;
                }
                break;
            case 13:  ///////////unlock rotation piston to send arms forward
                robot.setArmsForward();
                commandStep += 1;
                break;
            case 14: ///////////move arms forward

                if (!robot.getClimbSensorLeft() && countLeft == 0){
                    countLeft = 1;
                }

                if (robot.getClimbSensorLeft() && countLeft == 1){
                    leftArmPower = 0;
                    leftArrived = true;
                }
                else{
                    leftArmPower = defaultClimbPowerUp;
                }


                if (!robot.getClimbSensorRight() && countRight == 0){
                    countRight = 1;
                }

                if (robot.getClimbSensorRight() && countRight == 1){
                    rightArmPower = 0;
                    rightArrived = true;
                }
                else{
                    rightArmPower = defaultClimbPowerUp;
                }

                if (leftArrived && rightArrived){
                    countLeft = 0;
                    countRight = 0;
                    rightArmPower = 0;
                    leftArmPower = 0;
                    commandStep = 16; //////////skip over step 15
                    startingTime = System.currentTimeMillis();
                }
                break;
            case 15:///change to check with pitch and roll
                // actually don't even need this step :)
                if (robot.getPitch() >= -10){
                    //if (robot.armInContact()){
                    commandStep += 1;
                    leftArmPower = 0;
                    rightArmPower = 0;
                    rightArrived = false;
                    leftArrived = false;
                }
                else{
                    leftArmPower = -.1;
                    rightArmPower = -.1;
                }
            case 16://///////once in contact move arms back again with the piston and swiiiiing
                robot.setArmsBackward();
                if (System.currentTimeMillis() - startingTime >= 1000) {
                    commandStep += 1;//TODO:change back
                }
                break;
            case 17://////////go back to case 11 and repeat down to this step
                if (firstTime){
                    commandStep = 11;
                    countRight = 0;
                    countLeft = 0;
                    rightArrived = false;
                    leftArrived = false;
                    firstTime = false;
                }
                else{
                    commandStep += 1;
                    leftArmPower = 0;
                    rightArrived = false;
                    leftArrived = false;
                    rightArmPower = 0;
                }
                break;
            case 18:///////lift all the way up to be extra secure

                if (!robot.getClimbSensorLeft() && countLeft == 0){
                    countLeft = 1;
                }

                if (robot.getClimbSensorLeft() && countLeft == 1){
                    leftArmPower = 0;
                    leftArrived = true;
                }
                else{
                    leftArmPower = defaultClimbPowerDown;
                }


                if (!robot.getClimbSensorRight() && countRight == 0){
                    countRight = 1;
                }

                if (robot.getClimbSensorRight() && countRight == 1){
                    rightArmPower = 0;
                    rightArrived = true;
                }
                else{
                    rightArmPower = defaultClimbPowerDown;
                }

                if (robot.armInContact() && leftArrived && rightArrived){
                    countRight = 0;
                    countLeft = 0;
                    leftArmPower = 0;
                    rightArmPower = 0;
                    leftArrived = false;
                    rightArrived = false;
                    commandStep += 1;
                }
                break;
            case 19: ////////now we are done. If all goes well, we are on the traversal rung, if not, we no longer have a robot >;(
                leftArmPower = 0;
                rightArmPower = 0;
                break;


        }
        if (robot.getRoll() - level > leveltol){
            rightArmPower *= .8;
        }
        if (robot.getRoll() - level < - leveltol){
            leftArmPower *= .8;
        }

        robot.armPower(leftArmPower, rightArmPower);
    }

}
