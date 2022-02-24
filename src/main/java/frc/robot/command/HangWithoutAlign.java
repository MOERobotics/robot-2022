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

    double sensorDist = 12.0;
    double Tapetheta = 0;

    double correction;
    double currentYaw;

    long startingTime = 0;

    boolean leftSensor = false;
    boolean rightSensor = false;

    double lTraveled;

    double fwd = 71.6;
    PIDController PIDSteering;
    boolean tapeAlign;

    /////^^^^^^^^^^^Stuff for tapeAlign


    //////////////Now the real stuff
    double escapeHeight = 10;///TODO:what is this??
    boolean firstTime = true;
    int countLeft = 0;
    int countRight = 0;
    double leftArmPower = 0;
    double rightArmPower = 0;
    double defaultClimbPowerUp = -.5;
    double defaultClimbPowerDown = .5;
    boolean leftArrived = false;
    boolean rightArrived = false;
    double startHeightLeft = 0;
    double startHeightRight = 0;


    public void begin(GenericRobot robot){
        startingTime = System.currentTimeMillis();
        commandStep = -1;
        leftSensor = false;
        rightSensor = false;
        lTraveled = 0;
        fwd = 71.6;
        PIDSteering = new PIDController(robot.getPIDmaneuverP(), robot.getPIDmaneuverI(), robot.getPIDmaneuverD());
        tapeAlign = true;
        firstTime = true;
    }

    public void step(GenericRobot robot){
        SmartDashboard.putNumber("tapetheta", Tapetheta);
        SmartDashboard.putNumber("ltraveled", lTraveled);
        SmartDashboard.putNumber("fwd", fwd);
        SmartDashboard.putNumber("leftEncoderRaw", robot.encoderTicksLeftDrive());
        SmartDashboard.putNumber("rightEncoderRaw", robot.encoderTicksRightDrive());
        SmartDashboard.putBoolean("leftTapeSensor", robot.getFloorSensorLeft());
        SmartDashboard.putBoolean("rightTapeSensor", robot.getFloorSensorRight());
        SmartDashboard.putBoolean("leftCLimberSensor", robot.getClimbSensorLeft());
        SmartDashboard.putBoolean("rightClimberSensor", robot.getClimbSensorRight());
        SmartDashboard.putNumber("countLeft", countLeft);
        SmartDashboard.putNumber("countRight", countRight);
        SmartDashboard.putNumber("startDistance", startDistance);


        //////////////////////////start the real stuff now
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        switch (commandStep){
            case -1:
                commandStep += 1;
                break;
            case 0:///reset and enable PTO
                //reset encoders
                robot.turnOnPTO();
                robot.resetEncoders();
                countLeft = 0;
                countRight = 0;
                if (System.currentTimeMillis() - startingTime >= 50){
                    commandStep += 1;
                }

                break;
            case 1:  ///////////unlock rotation piston to send arms forward
                robot.setArmsForward();
                commandStep += 1;
                break;
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
                    commandStep += 1;

                }

                break;
            case 3:  ///////////unlock rotation piston to send arms forward
                robot.setArmsBackward();
                commandStep = 11;
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
                    commandStep = 30;
                    leftArrived = false;
                    rightArrived = false;
                    startHeightLeft = robot.armHeightLeft();
                    startHeightRight = robot.armHeightRight();
                    startingTime = System.currentTimeMillis();

                }
                break;
            case 30: //delay :)TODO:change
                if (System.currentTimeMillis() - startingTime >= 5000){
                    commandStep = 12;
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
                commandStep += 1;
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

            case 19: ////////now we are done. If all goes well, we are on the traversal rung, if not, we no longer have a robot >;(
                leftArmPower = 0;
                rightArmPower = 0;
                break;


        }
        robot.armPower(leftArmPower, rightArmPower);
    }

}
