package frc.robot.command;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.generic.GenericRobot;

public class Hang extends GenericCommand{


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

    double outerDistArc;
    double lTraveled;

    double fwd = 48;
    PIDController PIDSteering;
    int commandStep = -1;
    boolean tapeAlign;

    /////^^^^^^^^^^^Stuff for tapeAlign


    //////////////Now the real stuff
    double desiredHeight;
    double lowHeight;
    double escapeHeight;
    double getToBarHeight;
    boolean firstTime = true;


    public void begin(GenericRobot robot){
        startingTime = System.currentTimeMillis();
        commandStep = -1;
        leftSensor = false;
        rightSensor = false;
        lTraveled = 0;
        fwd = 48;
        PIDSteering = new PIDController(robot.getPIDmaneuverP(), robot.getPIDmaneuverI(), robot.getPIDmaneuverD());
        tapeAlign = true;
        firstTime = true;
    }

    public void step(GenericRobot robot){
        if (tapeAlign) {
            switch (commandStep) { /////////////tapeAlign Code
                case -1:
                    robot.resetEncoders();
                    robot.resetAttitude();

                    if (System.currentTimeMillis() >= startingTime + 100) {
                        commandStep += 1;
                    }
                    break;
                case 0:
                    startAngle = robot.getYaw();
                    PIDSteering.reset();
                    PIDSteering.enableContinuousInput(-180, 180);
                    commandStep += 1;
                    break;
                case 1:
                    correction = PIDSteering.calculate(robot.getYaw() - startAngle);
                    leftPower = defaultPower + correction; //didn't we stop doing this?
                    rightPower = defaultPower - correction;

                    if (!robot.getTapeSensorOne()) {
                        startDistance = robot.getDriveDistanceInchesLeft();
                        leftSensor = true;
                        commandStep += 1;
                    } else if (!robot.getTapeSensorTwo()) {
                        startDistance = robot.getDriveDistanceInchesLeft();
                        rightSensor = true;
                        commandStep += 1;
                    }
                    break;
                case 2:
                    correction = PIDSteering.calculate(robot.getYaw() - startAngle);
                    leftPower = defaultPower + correction; //confusion
                    rightPower = defaultPower - correction;

                    if (!rightSensor && !robot.getTapeSensorTwo()) {
                        differenceDistance = Math.abs(robot.getDriveDistanceInchesLeft() - startDistance);
                        Tapetheta = Math.atan(differenceDistance / sensorDist) * 180 / Math.PI;
                        outerDistArc = robot.getDriveDistanceInchesRight();
                        commandStep += 1;
                    } else if (!leftSensor && !robot.getTapeSensorOne()) {
                        differenceDistance = Math.abs(robot.getDriveDistanceInchesLeft() - startDistance);
                        Tapetheta = Math.atan(differenceDistance / sensorDist) * 180 / Math.PI;
                        outerDistArc = robot.getDriveDistanceInchesLeft();
                        commandStep = 4;
                    }
                    break;
                case 3://///////////////////////////////////////////////////////////////////skip this step
                    if (leftSensor) {
                        leftPower = defaultPower * .5;
                        rightPower = defaultPower * 2;
                    } else {
                        rightPower = defaultPower * .5;
                        leftPower = defaultPower * 2;
                    }
                    currentYaw = robot.getYaw();
                    if (Math.abs(Math.signum(currentYaw - startAngle) * (((Math.abs(currentYaw - startAngle) + 180) % 360) - 180)) >= Math.abs(Tapetheta)) {
                        if (rightSensor) {
                            outerDistArc = robot.getDriveDistanceInchesLeft() - outerDistArc;
                        } else {
                            outerDistArc = robot.getDriveDistanceInchesRight() - outerDistArc;
                        }
                        lTraveled = Math.abs(outerDistArc / (Tapetheta * Math.PI / 180) * Math.sin(Math.abs(Tapetheta * Math.PI / 180)));
                        commandStep += 1;
                    }///////////////////////////////////////this step is skipped
                    break;
                case 4:
                    if (leftSensor) {
                        currentYaw = startAngle - Tapetheta; //currentYaw = targetYaw because we are lazy
                    } else {
                        currentYaw = startAngle + Tapetheta; //currentYaw = targetYaw because we are lazy
                    }
                    PIDSteering.reset();
                    PIDSteering.enableContinuousInput(-180, 180);
                    startDistance = robot.getDriveDistanceInchesLeft();
                    commandStep += 1;
                    break;
                case 5:
                    correction = PIDSteering.calculate(robot.getYaw() - currentYaw);
                    leftPower = defaultPower + correction;
                    rightPower = defaultPower - correction;
                    if (Math.abs(robot.getDriveDistanceInchesLeft() - startDistance) >= (fwd - lTraveled)) {
                        leftPower = 0;
                        rightPower = 0;
                        commandStep += 1;
                    }
                    break;
                case 6: //adios amigos
                    leftPower = 0;
                    rightPower = 0;
                    tapeAlign = false;
                    commandStep = 0;
                    break;
            }
            robot.drivePercent(leftPower, rightPower);
        }
        else{////////////////////////////start the real stuff now
            switch (commandStep){
                case 0:///reset and enable PTO
                    robot.turnOnPTO();
                    commandStep += 1;
                    break;
                case 1: //////raise climber arms (skip 10 steps after in case we need to scoot/scoot
                    if (robot.armHeight() >= desiredHeight){
                        robot.armPower(0);
                        commandStep += 1;
                    }
                    else{
                        robot.raiseClimberArms();
                    }
                    break;
                case 2: //////disable PTO
                    robot.turnOffPTO();
                    commandStep += 1;
                    break;
                case 3: //////scoot back a lil
                    //go back, if in the right place, next step
                    if (robot.inTheRightPlace()){
                        commandStep += 1;
                        robot.armPower(0); // because this is really the drivetrain, it will work
                    }
                    else{
                        robot.lowerClimberArms(); //because this is really the drivetrain, it will work
                    }
                    break;
                case 4: //////enable PTO
                    robot.turnOnPTO();
                    commandStep = 11;
                    break;
                case 11: ////////lower climber arms
                    if (robot.armInContact() && robot.armHeight() <= lowHeight){
                        robot.armPower(0);
                        commandStep += 1;
                    }
                    else{
                        robot.lowerClimberArms();
                    }
                    break;
                case 12:  /////////////raise arms slightly
                    if (robot.armHeight() >= escapeHeight && !robot.armInContact()){
                        robot.armPower(0);
                        commandStep += 1;
                    }
                    else{
                        robot.raiseClimberArms();
                    }
                    break;
                case 13:  ///////////unlock rotation piston to send arms forward
                    robot.setArmsForward();
                    commandStep += 1;
                    break;
                case 14: ///////////move arms forward
                    if (robot.armHeight() >= getToBarHeight){
                        robot.armPower(0);
                        commandStep += 1;
                    }
                    else{
                        robot.raiseClimberArms();
                    }
                    break;
                case 15:
                    if (robot.armInContact()){
                        commandStep += 1;
                        robot.armPower(0);
                    }
                    else{
                        robot.lowerClimberArms();
                    }
                case 16://///////once in contact move arms back again with the piston and swiiiiing
                    robot.setArmsBackward();
                    commandStep += 1;
                    break;
                case 17://////////go back to case 11 and repeat down to this step
                    if (firstTime){
                        commandStep = 11;
                        firstTime = false;
                    }
                    else{
                        commandStep += 1;
                        robot.armPower(0);
                    }
                    break;
                case 19:///////lift all the way up to be extra secure
                    if (robot.armHeight() <= lowHeight && robot.armInContact()){
                        robot.armPower(0);
                        commandStep += 1;
                    }
                    else{
                        robot.lowerClimberArms();
                    }
                case 18: ////////now we are done. If all goes well, we are on the traversal rung, if not, we no longer have a robot >;(
                    robot.armPower(0);
                    break;

            }
        }
    }
}
