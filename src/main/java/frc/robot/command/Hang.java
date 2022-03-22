package frc.robot.command;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;

public class Hang extends GenericCommand{


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
    double escapeHeight = 6;///TODO:what is this??
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
    double topHeight = 27;

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
        SmartDashboard.putNumber("tapetheta", Tapetheta);
        SmartDashboard.putNumber("ltraveled", lTraveled);
        SmartDashboard.putNumber("fwd", fwd);
        SmartDashboard.putNumber("leftEncoderRaw", robot.encoderTicksLeftDriveA());
        SmartDashboard.putNumber("rightEncoderRaw", robot.encoderTicksRightDriveA());
        SmartDashboard.putBoolean("leftTapeSensor", robot.getFloorSensorLeft());
        SmartDashboard.putBoolean("rightTapeSensor", robot.getFloorSensorRight());
        SmartDashboard.putBoolean("leftCLimberSensor", robot.getClimbSensorLeft());
        SmartDashboard.putBoolean("rightClimberSensor", robot.getClimbSensorRight());
        SmartDashboard.putNumber("countLeft", countLeft);
        SmartDashboard.putNumber("countRight", countRight);
        SmartDashboard.putNumber("startDistance", startDistance);


        if (tapeAlign) {
            if (commandStep > -1) {

                if ((robot.getAlternateTurretAngle() <48) && (robot.getAlternateTurretAngle() > 42)){
                    turretPower = 0;
                    robot.raiseCollector();
                }
                else{
                    turretPower = -turretPIDController.calculate(robot.getAlternateTurretAngle() - 45);
                }
            }
            robot.setTurretPowerPct(turretPower);
            switch (commandStep) { /////////////tapeAlign Code
                case -1:
                    robot.resetEncoders();
                    robot.resetAttitude();
                    turretPIDController.disableContinuousInput();

                    if (System.currentTimeMillis() >= startingTime + 100) {
                        System.out.print("We are going to step 0 of the align at ");
                        System.out.println(System.currentTimeMillis()%1000000);
                        commandStep += 1;
                    }
                    break;
                case 0:
                    startAngle = robot.getYaw();
                    startDistance = robot.getDriveDistanceInchesLeft();
                    PIDSteering.reset();
                    PIDSteering.enableContinuousInput(-180, 180);
                    System.out.print("We are going to step 1 of the align at ");
                    System.out.println(System.currentTimeMillis()%1000000);
                    commandStep += 1;//TODO:change back
                    break;

                case 1:
                    correction = PIDSteering.calculate(robot.getYaw() - startAngle);
                    leftPower = defaultPower + correction; //didn't we stop doing this?
                    rightPower = defaultPower - correction;

                    if (!robot.getFloorSensorLeft() && !robot.getFloorSensorRight()){
                        Tapetheta = 0;
                        System.out.print("Already straight, we are going to step 3 of the align at ");
                        System.out.println(System.currentTimeMillis()%1000000);
                        commandStep = 3;
                    }
                    else if (!robot.getFloorSensorLeft()) {
                        startDistance = robot.getDriveDistanceInchesLeft();
                        leftSensor = true;
                        System.out.print("Pointed to the right, We are going to step 2 of the align at ");
                        System.out.println(System.currentTimeMillis()%1000000);
                        commandStep += 1;
                    } else if (!robot.getFloorSensorRight()) {
                        startDistance = robot.getDriveDistanceInchesLeft();
                        rightSensor = true;
                        System.out.print("Pointed to the left, we are going to step 2 of the align at ");
                        System.out.println(System.currentTimeMillis()%1000000);
                        commandStep += 1;
                    }
                    break;
                case 2:
                    correction = PIDSteering.calculate(robot.getYaw() - startAngle);
                    leftPower = defaultPower + correction; //confusion
                    rightPower = defaultPower - correction;

                    if (!rightSensor && !robot.getFloorSensorRight()) {
                        differenceDistance = Math.abs(robot.getDriveDistanceInchesLeft() - startDistance);
                        Tapetheta = (Math.atan(differenceDistance / sensorDist) * 180 / Math.PI);
                        System.out.print("We are going to step 3 of the align at ");
                        System.out.println(System.currentTimeMillis()%1000000);
                        commandStep += 1;
                    } else if (!leftSensor && !robot.getFloorSensorLeft()) {
                        differenceDistance = Math.abs(robot.getDriveDistanceInchesLeft() - startDistance);
                        Tapetheta = (Math.atan(differenceDistance / sensorDist) * 180 / Math.PI);
                        System.out.print("We are going to step 3 of the align at ");
                        System.out.println(System.currentTimeMillis()%1000000);
                        commandStep += 1;
                    }
                    break;

                case 3:
                    robot.setArmsForward();
                    if (leftSensor) {
                        currentYaw = startAngle - Tapetheta; //currentYaw = targetYaw because we are lazy
                    } else {
                        currentYaw = startAngle + Tapetheta; //currentYaw = targetYaw because we are lazy
                    }
                    PIDSteering.reset();
                    PIDSteering.enableContinuousInput(-180, 180);
                    startDistance = robot.getDriveDistanceInchesLeft();
                    System.out.print("We are going to step 4 of the align at ");
                    System.out.println(System.currentTimeMillis()%1000000);
                    commandStep += 1;
                    break;
                case 4:
                    correction = PIDSteering.calculate(robot.getYaw() - currentYaw);
                    leftPower = defaultPower + correction;
                    rightPower = defaultPower - correction;
                    if (Math.abs(robot.getDriveDistanceInchesLeft() - startDistance) >= (fwd-10)){
                        double ramp = rampDown(defaultPower, .1, startDistance, 10, robot.getDriveDistanceInchesLeft(), fwd);
                        leftPower = ramp;
                        rightPower = ramp;
                    }
                    if (Math.abs(robot.getDriveDistanceInchesLeft() - startDistance) >= (fwd)) {
                        leftPower = 0;
                        rightPower = 0;
                        System.out.print("We are going to step 5 of the align at ");
                        System.out.println(System.currentTimeMillis()%1000000);
                        commandStep += 1;
                    }
                    break;
                case 5: //adios amigos
                    leftPower = 0;
                    rightPower = 0;
                    tapeAlign = false;
                    System.out.print("We are going to start the climb at ");
                    System.out.println(System.currentTimeMillis()%1000000);
                    commandStep = -1;
                    startingTime = System.currentTimeMillis();
                    break;
                    /////////TODO:Put arms forward
            }
            robot.drivePercent(leftPower, rightPower);
        }
        else{////////////////////////////start the real stuff now
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            switch (commandStep){
                case -1:
                    commandStep += 1;
                    System.out.print("We are going to step 0 of the climb at ");
                    System.out.println(System.currentTimeMillis()%1000000);
                    break;
                case 0:///reset and enable PTO
                    //reset encoders
                    robot.turnOnPTO();
                    robot.resetEncoders();
                    leftArmPower = 0;
                    rightArmPower = 0;
                    countLeft = 0;
                    countRight = 0;
                    startHeightLeft = robot.armHeightLeft();
                    startHeightRight = robot.armHeightRight();
                    if (System.currentTimeMillis() - startingTime >= 2000){
                        SmartDashboard.putNumber("driveOutputCurrent", robot.getDriveCurrent());
                        System.out.print("We are going to step 2 of the climb at ");
                        System.out.println(System.currentTimeMillis()%1000000);
                        commandStep = 2; ///TODO: fix numbering
                    }

                    break;

                case 2: //////raise climber arms (skip 10 steps after in case we need to scoot/scoot

                    if (robot.getClimbSensorLeft() && countLeft == 0){
                        startHeightLeft = robot.armHeightLeft();
                        countLeft = 1;
                    }

                    if ((robot.armHeightLeft() - startHeightLeft) >= topHeight){
                        leftArmPower = 0;
                        leftArrived = true;
                    }
                    else{
                        leftArmPower = defaultClimbPowerUp;
                    }


                    if (robot.getClimbSensorRight() && countRight == 0){
                        countRight = 1;
                        startHeightRight = robot.armHeightRight();
                    }

                    if ((robot.armHeightRight() - startHeightRight) > topHeight){
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
                        System.out.print("We are going to step 3 of the climb at ");
                        System.out.println(System.currentTimeMillis()%1000000);
                        commandStep += 1;

                    }

                    break;
                case 3:  ///////////unlock rotation piston to send arms back
                    robot.setArmsBackward();
                    if (System.currentTimeMillis() - startingTime >= 1000) {
                        System.out.print("We are going to step 11 of the climb at ");
                        System.out.println(System.currentTimeMillis()%1000000);
                        commandStep = 11;
                    }
                    break;

                case 11: ////////lower climber arms

                    if (robot.getClimbSensorLeft() && countLeft == 0){
                        countLeft = 1;
                    }

                    if (!robot.getClimbSensorLeft() && countLeft == 1){
                        leftArmPower = 0;
                        leftArrived = true;
                    }
                    else{
                        leftArmPower = defaultClimbPowerDown;
                    }


                    if (robot.getClimbSensorRight() && countRight == 0){
                        countRight = 1;
                    }

                    if (!robot.getClimbSensorRight() && countRight == 1){
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
                        System.out.print("We are going to step 12 of the climb at ");
                        System.out.println(System.currentTimeMillis()%1000000);
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
                        System.out.print("We are going to step 13 of the climb at ");
                        System.out.println(System.currentTimeMillis()%1000000);
                        commandStep += 1;
                    }
                    break;
                case 13:  ///////////unlock rotation piston to send arms forward
                    robot.setArmsForward();
                    System.out.print("We are going to step 14 of the climb at ");
                    System.out.println(System.currentTimeMillis()%1000000);
                    commandStep += 1;
                    break;
                case 14: ///////////bring arms up


                    if (robot.armHeightLeft() - startHeightLeft >= topHeight){
                        leftArmPower = 0;
                        leftArrived = true;
                    }
                    else{
                        leftArmPower = defaultClimbPowerUp;
                    }


                    if (robot.armHeightRight() - startHeightRight >= topHeight){
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
                        System.out.print("We are going to step 16 of the climb at ");
                        System.out.println(System.currentTimeMillis()%1000000);
                        commandStep = 16; //////////skip over step 15
                        startingTime = System.currentTimeMillis();
                    }
                    break;

                case 16://///////once in contact move arms back again with the piston and swiiiiing
                    robot.setArmsBackward();
                    if (System.currentTimeMillis() - startingTime >= 1000) {
                        System.out.print("We are going to step 17 of the climb at ");
                        System.out.println(System.currentTimeMillis()%1000000);
                        commandStep += 1;//TODO:change back
                    }
                    break;
                case 17://////////go back to case 11 and repeat down to this step
                    if (firstTime){
                        System.out.print("We are going to step 11 of the climb at ");
                        System.out.println(System.currentTimeMillis()%1000000);
                        commandStep = 11;
                        countRight = 0;
                        countLeft = 0;
                        rightArrived = false;
                        leftArrived = false;
                        firstTime = false;
                    }
                    else{
                        System.out.print("We are going to step 18 of the climb at ");
                        System.out.println(System.currentTimeMillis()%1000000);
                        commandStep += 1;
                        leftArmPower = 0;
                        rightArrived = false;
                        leftArrived = false;
                        rightArmPower = 0;
                        startHeightLeft = robot.armHeightLeft();
                        startHeightRight = robot.armHeightRight();
                    }
                    break;
                case 18:///////lift part of the way up to be extra secure

                    if (Math.abs(robot.armHeightLeft()-startHeightLeft) >= escapeHeight + 10){
                        leftArmPower = 0;
                        leftArrived = true;
                    }
                    else{
                        leftArmPower = defaultClimbPowerDown;
                    }
                    if (Math.abs(robot.armHeightRight()-startHeightRight) >= escapeHeight + 10){
                        rightArmPower = 0;
                        rightArrived = true;
                    }
                    else{
                        rightArmPower = defaultClimbPowerDown;
                    }
                    if (rightArrived && leftArrived){
                        rightArmPower = 0;
                        leftArmPower = 0;
                        rightArrived = false;
                        leftArrived = false;
                        System.out.print("We are going to step 19 of the climb at ");
                        System.out.println(System.currentTimeMillis()%1000000);
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
        /*if (Math.abs(robot.getRoll()-level) >= 3*leveltol){
            rightArmPower = 0;
            leftArmPower = 0;
        }*/
        robot.armPower(leftArmPower, rightArmPower);
        }
    }
}
