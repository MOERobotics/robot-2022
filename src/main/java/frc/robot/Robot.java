// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.*;
import frc.robot.autonomous.GenericAutonomous;
import frc.robot.command.*;
import frc.robot.generic.*;

import java.util.Arrays;
import java.util.Map;
import java.util.function.Function;
import java.util.stream.Collectors;

public class Robot extends TimedRobot {

  //Instantiate autonomous once, don't create unnecessary duplicates
  //Add new Autos here when they're authored
  public static final GenericAutonomous
          autoArc = new AutoArc5Ball(),
          ATerminalReturn = new BallAtoTerminalReturn(),
          simpleBTerminal = new BallBtoTerminal(),
          simpleCTerminal = new BallCtoTerminal(),
          CTerminalReturn = new BallCtoTerminalReturn(),
          BTerminalReturn = new BallBtoTerminalReturn(),
          simpleB         = new BallSimpleB(),
          calibration = new Calibration(),
          shortRun = new ShortRun();

  GenericRobot robot = new Lightning();
  Joystick joystick = new Joystick(0);
  GenericCommand command = new Hang();
  Joystick xbox = new Joystick(1);
  GenericAutonomous autonomous = calibration;
  GenericCommand testHang = new HangWithoutAlign();

  Lidar asdf = new Lidar();



  int averageTurretXSize = 2;
  double[] averageX = new double[averageTurretXSize];


  int counter = 0;
  double average;
  double turretPower;
  boolean hang = false;
  int countHang = 0;
  int countShoot = 0;
  boolean shoot = false;
  boolean reset = true;
  double maxCurrentLeftA = 0;
  double maxCurrentLeftB = 0;
  double maxCurrentRightA = 0;
  double maxCurrentRightB = 0;
  boolean ActuallyHanging = false;

  double driveLeft = 0;
  double driveRight = 0;
  double joystickX;
  double joystickY;
  double cutoff = 0.05;
  double scaleFactor = 1.0;

  int leftAxis = 1;
  int rightAxis = 5;
  double tolerance = 0.8;
  double defaultClimbPower = 0.2;

  double shooterTargetRPM = 0;

  double defCollectorPower = 1;
  double defIndexerPower = 1;
  double curCollector;
  double curIndexer;

  double targetRPM = 0;
  double turretPitch = 0;

  boolean turnTo45 = false;
  boolean turnTo225 = false;

  boolean isShooting = false;


  PIDController turretPIDController;

  boolean armReset = false;
  double timerForPTO;
  boolean delayLeft = false;
  boolean delayRight = false;
  double leftTime;
  double rightTime;



  @Override
  public void robotInit() {
    System.out.println("Klaatu barada nikto");
    robot.setTurretPitchPosition(0);
    asdf.start();
  }

  @Override
  public void robotPeriodic() {

    if (countShoot == 0){
      isShooting = false;
    }
    else{
      isShooting = robot.isReadyToShoot();
    }

    if (robot.getLeftACurrent() > maxCurrentLeftA) {
      maxCurrentLeftA = robot.getLeftACurrent();
    }
    if (robot.getLeftBCurrent() > maxCurrentLeftB) {
      maxCurrentLeftB = robot.getLeftBCurrent();
    }
    if (robot.getRightACurrent() > maxCurrentRightA) {
      maxCurrentRightA = robot.getRightACurrent();
    }
    if (robot.getRightBCurrent() > maxCurrentRightB) {
      maxCurrentRightB = robot.getRightBCurrent();
    }


    SmartDashboard.putNumber("XBOX AXIS DEBUG - 0 ", xbox.getRawAxis(0));
    SmartDashboard.putNumber("XBOX AXIS DEBUG - 1 ", xbox.getRawAxis(1));
    SmartDashboard.putNumber("XBOX AXIS DEBUG - 2 ", xbox.getRawAxis(2));
    SmartDashboard.putNumber("XBOX AXIS DEBUG - 3 ", xbox.getRawAxis(3));

    SmartDashboard.putNumber("LeftACurrCurrent", robot.getLeftACurrent());
    SmartDashboard.putNumber("LeftBCurrCurrent", robot.getLeftBCurrent());
    SmartDashboard.putNumber("RightACurrCurrent", robot.getRightACurrent());
    SmartDashboard.putNumber("RightBCurrCurrent", robot.getRightBCurrent());
    SmartDashboard.putNumber("LeftACurrentMax", maxCurrentLeftA);
    SmartDashboard.putNumber("LeftBCurrentMax", maxCurrentLeftB);
    SmartDashboard.putNumber("RightACurrentMax", maxCurrentRightA);
    SmartDashboard.putNumber("RightBCurrentMax", maxCurrentRightB);

    //SmartDashboard.putNumber("LimelightX", robot.getTargetX());
    //SmartDashboard.putNumber("LimelightY", robot.getTargetY());
    SmartDashboard.putNumber("LimelightArea", robot.getTargetArea());

    SmartDashboard.putNumber("Drive left pct", robot.getDriveLeftPercentage());
    SmartDashboard.putNumber("Drive right pct", robot.getDriveRightPercentage());
    SmartDashboard.putNumber("Drive left rpm", robot.getDriveLeftRPM());
    SmartDashboard.putNumber("Drive right rpm", robot.getDriveRightRPM());

    SmartDashboard.putNumber("Left encoder Ticks A", robot.encoderTicksLeftDriveA());
    SmartDashboard.putNumber("Right encoder Ticks A", robot.encoderTicksRightDriveA());

    SmartDashboard.putNumber("Left encoder Ticks B", robot.encoderTicksLeftDriveB());
    SmartDashboard.putNumber("Right encoder Ticks B", robot.encoderTicksRightDriveB());

    SmartDashboard.putNumber("Left encoder Inches", robot.getDriveDistanceInchesLeft());
    SmartDashboard.putNumber("Right encoder Inches", robot.getDriveDistanceInchesRight());

    SmartDashboard.putNumber("Yaw", robot.getYaw());
    SmartDashboard.putNumber("Pitch", robot.getPitch());
    SmartDashboard.putNumber("Rollll", robot.getRoll());
    SmartDashboard.putNumber("Linear speed", robot.getLinearVelocity());

    SmartDashboard.putBoolean("Have upper cargo? (indexer reverse)", robot.getUpperCargo());
    SmartDashboard.putBoolean("Have lower cargo? (indexer forward)", robot.getLowerCargo());

    SmartDashboard.putBoolean("Trip left climb sensor? (leftB reverse)", robot.getClimbSensorLeft());
    SmartDashboard.putBoolean("Trip right climb sensor? (rightA reverse)", robot.getClimbSensorRight());

    SmartDashboard.putBoolean("Trip left floor sensor? (leftB forward)", robot.getFloorSensorLeft());
    SmartDashboard.putBoolean("Trip right floor sensor? (rightA forward)", robot.getFloorSensorRight());

    //SmartDashboard.putBoolean("Has detected cargo?", robot.hasFoundCargo());

    SmartDashboard.putNumber("Collector intake power", robot.getCollectorIntakePercentage());
    SmartDashboard.putNumber("Indexer   intake power", robot.getIndexerIntakePercentage());

    //SmartDashboard.getBoolean("Sees target?", robot.isTargetFound());

    SmartDashboard.putBoolean("Vision target found", robot.isTargetFound());
    SmartDashboard.putNumber("Vision target x", robot.getTargetX());
    SmartDashboard.putNumber("Vision target Average", average);
    SmartDashboard.putNumber("Vision target y", robot.getTargetY());
    SmartDashboard.putNumber("Vision target angle", robot.getTargetAngle());
    SmartDashboard.putNumber("Vision target dist", robot.getTargetDistance());

    //SmartDashboard.putNumber("Turret direction angle ticks", robot.getTurretAngle());
    //SmartDashboard.putNumber("Turret direction angle degrees", robot.getTurretAngleDegrees());
    SmartDashboard.putNumber("Alternate turret angle degrees", robot.getAlternateTurretAngle());

    SmartDashboard.putNumber("Turret direction motor pct", robot.getTurretPowerPct());

    //SmartDashboard.putNumber("Turret direction motor pct", robot.getTurretPowerPct());

    SmartDashboard.putNumber("Turret pitch position", robot.getTurretPitchPosition());
    SmartDashboard.putNumber("Turret pitch motor pct", robot.getTurretPitchPowerPct());

    SmartDashboard.putNumber("Shooter top motor pct", robot.getShooterPowerPctTop());
    SmartDashboard.putNumber("Shooter bottom motor pct", robot.getShooterPowerPctBottom());

    SmartDashboard.putNumber("Shooter top motor rpm", robot.getShooterRPMTop());
    SmartDashboard.putNumber("Shooter bottom motor rpm", robot.getShooterRPMBottom());
    SmartDashboard.putNumber("Shooter C motor rpm", robot.getShooterRPMC());

    SmartDashboard.putNumber("Shooter calculate distance", robot.getShooterTargetDistance());
    SmartDashboard.putNumber("Shooter calculate height", robot.getShooterTargetHeight());
    SmartDashboard.putNumber("Shooter target RPM", robot.getShooterTargetRPM());

    SmartDashboard.putNumber("Shooter Ready Timer", robot.getShootReadyTimer());
    SmartDashboard.putBoolean("Shooter is Ready?", robot.isReadyToShoot());
    SmartDashboard.putBoolean("Shooter is Actively Firing?", robot.isActivelyShooting());


    SmartDashboard.putBoolean("Is PTO set to climb arms?", robot.getPTOState());

    SmartDashboard.putNumber("Joystick raw X", joystick.getX());
    SmartDashboard.putNumber("Joystick raw Y", joystick.getY());

    SmartDashboard.putNumber("Autonomous Step", autonomous.autonomousStep);
    SmartDashboard.putString("Autonomous Program", autonomous.getClass().getName());

    SmartDashboard.putNumber("leftEncoderRaw", robot.encoderTicksLeftDriveA());
    SmartDashboard.putNumber("rightEncoderRaw", robot.encoderTicksRightDriveA());
    SmartDashboard.putBoolean("leftTapeSensor", robot.getFloorSensorLeft());
    SmartDashboard.putBoolean("rightTapeSensor", robot.getFloorSensorRight());
    SmartDashboard.putBoolean("leftCLimberSensor", robot.getClimbSensorLeft());
    SmartDashboard.putBoolean("rightClimberSensor", robot.getClimbSensorRight());

    SmartDashboard.putNumber("rightArmEncoder", robot.armHeightRight());
    SmartDashboard.putNumber("leftArmEncoder", robot.armHeightLeft());

    SmartDashboard.putBoolean("hang", hang);
    SmartDashboard.putNumber("count", countHang);

    SmartDashboard.putNumber("CommandStep", command.commandStep);

    SmartDashboard.putBoolean("Robot is Shooting?", isShooting);


    SmartDashboard.putNumber("Lidar A", asdf.getDistance(0));
    SmartDashboard.putNumber("Lidar B", asdf.getDistance(1));
    SmartDashboard.putNumber("Lidar C", asdf.getDistance(2));
    SmartDashboard.putNumber("Lidar D", asdf.getDistance(3));

  }

  @Override
  public void autonomousInit() {
    autonomous.autonomousInit(robot);
  }

  @Override
  public void autonomousPeriodic() {
    autonomous.autonomousPeriodic(robot);
  }

  @Override
  public void teleopInit() {
    shoot = false;
    countShoot = 0;
    turretPitch = 0;
    turretPIDController = new PIDController(robot.turretPIDgetP(), robot.turretPIDgetI(), robot.turretPIDgetD());
    hang = false;
    countHang = 0;
    xbox.getRawButtonPressed(3);
    turnTo45 = false;
    turnTo225 = false;
  }

  @Override
  public void teleopPeriodic() {
    turretPower = 0;


    switch (POVDirection.getDirection(xbox.getPOV())) {
      case NORTH: //MEDIUM SHOT RANGE
        targetRPM = 2611;
        turretPitch = 0.25;
        break;
      case EAST:
        targetRPM = 1700;
        turretPitch = 1.0;
        break;
      case SOUTH: ////CLOSE SHOT--> collector out
        targetRPM = 3400*.7;
        turretPitch = 0.00;
        turnTo225 = true;
        turnTo45 = false;
        break;
      case WEST:
        targetRPM = 2340; //////////collector facing
        turretPitch = 0.08;
        turnTo45 = true;
        turnTo225 = false;
        break;
    }

    if (joystick.getRawButtonPressed(8)) {
      countHang = (countHang + 1) % 2;
    }

    if (countHang == 1) {
      hang = true;
    } else {
      hang = false;
    }

    if (!hang && !armReset) {
      reset = true;
      robot.turnOffPTO();

      //////////////////////////////////////////////////DRIVETRAIN CONTROL

      if (!robot.getPTOState()) {

        if(joystick.getRawButton(1)){
          driveLeft = .4;
          driveRight = .4 ;
        }
        else if(joystick.getRawButton(2)){
          driveLeft = -.4;
          driveRight = -.4;
        }
        else if(joystick.getRawButton(3)){
          //pivot left
          driveLeft = -.4;
          driveRight = .4;
        }
        else if(joystick.getRawButton(4)){
          //pivot right
          driveLeft = .4;
          driveRight = -.4;
        }
        else{
          joystickX = joystick.getX();
          joystickY = -joystick.getY();

          if (joystickY > -cutoff && joystickY < cutoff) {
            joystickY = 0;
          }
          if (joystickX > -cutoff && joystickX < cutoff) {
            joystickX = 0;
          }

          driveLeft = (joystickY + joystickX) * scaleFactor;
          driveRight = (joystickY - joystickX) * scaleFactor;
        }

      }

      //////////////////////////////////////////////DRIVETRAIN CONTROL ENDS


      //////////////////////////////////////////////////////////TURRET CONTROL

      counter = (counter + 1) % averageTurretXSize;
      averageX[counter] = robot.getTargetX();

      average = 0;
      for (int i = 0; i < averageTurretXSize; i++)
        average += averageX[i];
      average /= averageTurretXSize;

      if ((xbox.getRawAxis(2) > 0.10) && robot.isTargetFound()) { ////////////AUTO-AIM
        turnTo45 = false;
        turnTo225 = false;
        turretPower = turretPIDController.calculate(average);
        targetRPM = robot.findShooterRPM();
        turretPitch = robot.findShooterPitch();

      }

      else {
        turretPIDController.reset();
        if (xbox.getRawButton(6)) {
          turretPower = -0.45;
          turnTo45 = false;
          turnTo225 = false;
        } else if (xbox.getRawButton(5)) {
          turretPower = 0.45;
          turnTo45 = false;
          turnTo225 = false;
        }
        else if(turnTo45) {
            turretPower = -turretPIDController.calculate(robot.getAlternateTurretAngle()-45);
          }
        else if (turnTo225){
            turretPower = -turretPIDController.calculate(robot.getAlternateTurretAngle()-225);
        }
        else {
          turretPower = 0.0;
        }
      }


      /////////////////////////////////////////////////////TURRET CONTROL ENDS


      /////////////////////////////////////////////////////CLIMBER WHEN PTO

      if (joystick.getRawButtonPressed(9) && joystick.getRawButtonPressed(10)) {
        armReset = true;
        timerForPTO = System.currentTimeMillis();
      }
      /////////////////////////////////////////////////////CLIMBER CODE ENDS


      //////////////////////////////////////////////////////////SHOOTER CODE BEGINS

      if (xbox.getRawButtonPressed(3)) {
        countShoot = (countShoot + 1) % 2;
      }
      if (countShoot == 0){
        shoot = false;
      }
      else{
        shoot = true;
      }

      if (xbox.getRawAxis(1) > .1){
        targetRPM += 5;
      }
      else if (xbox.getRawAxis(1) < -.1){
        targetRPM -= 5;
      }

      if (shoot){
        shooterTargetRPM = targetRPM;
      } else {
        shooterTargetRPM = 0;
      }


      if (xbox.getRawButton(1)) {
        if (robot.isReadyToShoot()) {
          robot.setActivelyShooting(true);
        } else {
          robot.setActivelyShooting(false);
        }
      } else {
        robot.setActivelyShooting(false);
      }

      //force override rpm check
      if (joystick.getRawButton(7)) {
        robot.setActivelyShooting(true);
      }
      ////////////////////////////////////////////////////////////SHOOTER CODE ENDS


      ///////////////////////////////////////////////////////////ACTUATOR STUFF

      double deadzone = 0.5;
      double rJoyRY = xbox.getRawAxis(5);
      if (rJoyRY > -deadzone && rJoyRY < deadzone) rJoyRY = 0;

      if (rJoyRY > deadzone) {
        turretPitch = robot.getTurretPitchPosition() + .002;
      }

      if (rJoyRY < -deadzone) {
        turretPitch = robot.getTurretPitchPosition() - .002;
      }


      /////////////////////////////////////////////////////////////////ACTUATOR STUFF ENDS


      /////////////////////////////////////////////////////////COLLECTOR CONTROLS


      //button 2 = bottom center button
      if (xbox.getRawButton(2)) {
        if (!robot.getUpperCargo()) {
          //consumes ball
          curCollector = defCollectorPower;
          curIndexer = defIndexerPower;
        } else {
          curIndexer = 0;
          if (!robot.getLowerCargo()) {
            curCollector = defCollectorPower;
          } else {
            curCollector = 0;
          }
        }
        if (robot.isActivelyShooting()) {
          curIndexer = defIndexerPower;
        }
      } else if (xbox.getRawButton(4)) {
        //barfs out ball
        curCollector = -defCollectorPower;
        curIndexer = -defIndexerPower;
      } else {
        curCollector = 0;
        curIndexer = 0;
      }

      //Cargo is on upper sensor and we want to yeet it: indexer needs to push it past sensor
      if (robot.isActivelyShooting() && robot.getUpperCargo()) {
        curIndexer = defIndexerPower;
      }

      /* Some code for setting up the collector after flight check. */
      double flightCheckTol = 5.0;
      double storagePosition = 317;
      
      if (joystick.getRawButton(13)) {
        if (robot.getAlternateTurretAngle()>storagePosition+flightCheckTol)
        {
          turretPower = 0.4;
        }
        else if (robot.getAlternateTurretAngle()<storagePosition-flightCheckTol)
        {
          turretPower = -0.4;
        }
        else
        {
          turretPower = 0.0;
        }
      }

      if (joystick.getRawButton(11) && joystick.getRawButton(12) &&
              (Math.abs(robot.getAlternateTurretAngle()-storagePosition)<flightCheckTol) )
      {
        robot.raiseCollector();
      }

      if (joystick.getRawButton(16)){
        robot.lowerCollector();
      }


      //////////////////////////////////////////////////COLLECTOR LOGIC ENDS


      ///////////////////////////////////////////////////POWER SETTERS

      robot.drivePercent(driveLeft, driveRight);
      robot.setShooterRPM(shooterTargetRPM, shooterTargetRPM);
      robot.setTurretPitchPosition(turretPitch);
      robot.setTurretPowerPct(turretPower);
      robot.setCollectorIntakePercentage(curCollector);
      robot.setIndexerIntakePercentage(curIndexer);

      //////////////////////////////////////////////////POWER SETTERS END

    }
    if (armReset){
      robot.turnOnPTO();
      if (armReset && (System.currentTimeMillis() - timerForPTO)>=2000){
        driveLeft = -.2;
        driveRight = -.2;
        if (!robot.getClimbSensorRight() && !delayRight){
          delayRight = true;
          rightTime = System.currentTimeMillis();
        }
        if (!robot.getClimbSensorLeft() && !delayLeft){
          delayLeft = true;
          leftTime = System.currentTimeMillis();
        }
        if (!robot.getClimbSensorLeft() && (System.currentTimeMillis() - leftTime >= 100)){
          driveLeft = 0;
        }
        if (!robot.getClimbSensorRight() && (System.currentTimeMillis() - rightTime >= 100)){
          driveRight = 0;
        }

        if (!robot.getClimbSensorLeft() && !robot.getClimbSensorRight()
                && (System.currentTimeMillis() - leftTime >= 100)
                && (System.currentTimeMillis() - rightTime >= 100)){
          robot.turnOffPTO();
          armReset = false;
          delayRight = false;
          delayLeft = false;
        }
        robot.drivePercent(driveLeft, driveRight);
      }

    }
    if (hang) {
      ///////////////////////////////////////////RUN AUTO-CLIMB
      ActuallyHanging = true;
      SmartDashboard.putBoolean("we really are hanging", ActuallyHanging);
      if (reset) {
        command.begin(robot);
        reset = false;
      }
      command.step(robot);
    }

  }


  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    joystick.getRawButtonPressed(10);
    joystick.getRawButtonPressed(9);
    xbox.getRawButtonPressed(3);
    joystick.getRawButtonPressed(8);
    hang = false;
    countHang = 0;
    if (joystick.getRawButton(1)) {
      robot.resetAttitude();
      robot.resetEncoders();
    }

    if (joystick.getRawButton(4)) autonomous = autoArc;
    if (joystick.getRawButton(5)) autonomous = ATerminalReturn;
    if (joystick.getRawButton(6)) autonomous = BTerminalReturn;
    if (joystick.getRawButton(9)) autonomous = CTerminalReturn;
    if (joystick.getRawButton(7)) autonomous = shortRun;

  }

  @Override
  public void testInit() {
    joystick.getRawButtonPressed(4);
  }

  @Override
  public void testPeriodic() {
    LiveWindow.setEnabled(false);
    double pitchChange = 0;
    if (xbox.getRawButton(1)) {
      pitchChange = 0.02;
    } else if (xbox.getRawButton(2)) {
      pitchChange = -0.02;
    } else {
      pitchChange = 0;
    }
    double newPos = robot.getTurretPitchPosition() + pitchChange;
    if (newPos < 0) newPos = 0;
    if (newPos > 1) newPos = 1;

    robot.setTurretPitchPosition(newPos);

    double driveX = joystick.getX();
    double driveY = -joystick.getY();

    //joystick deaden: yeet smol/weird joystick values when joystick is at rest
    double cutoff = 0.05;
    if (driveX > -cutoff && driveX < cutoff) driveX = 0;
    if (driveY > -cutoff && driveY < cutoff) driveY = 0;

    //moved this to after joystick deaden because deaden should be focused on the raw joystick values
    double scaleFactor = 1.0;

    if (!robot.getPTOState()) {
      robot.drivePercent(
              (driveY + driveX) * scaleFactor,
              (driveY - driveX) * scaleFactor
      );
    }

    int leftAxis = 1;
    int rightAxis = 5;
    double tolerance = 0.8;
    double drivePower = 0.2;

    if (joystick.getRawButton(12)) robot.setTurretPowerPct(0.2);
    else if (joystick.getRawButton(15)) robot.setTurretPowerPct(-0.2);
    else robot.setTurretPowerPct(0.0);

    double driveLeft = 0;
    double driveRight = 0;

    if (robot.getPTOState()) {
      if (xbox.getRawAxis(leftAxis) > tolerance) {
        System.out.print("LeftA Motor Current Up- ");
        System.out.println(robot.getLeftACurrent());
        System.out.print("LeftB Motor Current Up- ");
        System.out.println(robot.getLeftBCurrent());
        driveLeft = drivePower;
      } else if (xbox.getRawAxis(leftAxis) < -tolerance) {
        driveLeft = -drivePower;
        System.out.print("LeftA Motor Current Down- ");
        System.out.println(robot.getLeftACurrent());
        System.out.print("LeftB Motor Current Down- ");
        System.out.println(robot.getLeftBCurrent());
        if (!robot.getClimbSensorLeft()){
          driveLeft = 0;
        }
      }

      if (xbox.getRawAxis(rightAxis) > tolerance) {
        driveRight = drivePower;
        System.out.print("RightA Motor Current Up- ");
        System.out.println(robot.getRightACurrent());
        System.out.print("RightB Motor Current Up- ");
        System.out.println(robot.getRightBCurrent());
      } else if (xbox.getRawAxis(rightAxis) < -tolerance) {
        driveRight = -drivePower;
        System.out.print("RightA Motor Current Down- ");
        System.out.println(robot.getRightACurrent());
        System.out.print("RightB Motor Current Down- ");
        System.out.println(robot.getRightBCurrent());
        if (!robot.getClimbSensorRight()){
          driveRight = 0;
        }
      }
      robot.drivePercent(driveLeft, driveRight);
    }

    //note to self: buttons control mirrored joystick setting
    if (joystick.getRawButton(11)) {
      robot.setCollectorIntakePercentage(1);
      robot.setIndexerIntakePercentage(1);
    } else if (joystick.getRawButton(16)) {
      robot.setCollectorIntakePercentage(-1);
      robot.setIndexerIntakePercentage(-1);
    } else {
      robot.setCollectorIntakePercentage(0);
      robot.setIndexerIntakePercentage(0);
    }

    if (joystick.getRawButton(12)) robot.setTurretPowerPct(0.2);
    else if (joystick.getRawButton(15)) robot.setTurretPowerPct(-0.2);
    else robot.setTurretPowerPct(0.0);


    if (joystick.getRawButton(13)) {
      robot.setShooterPowerPct(0.2, 0.2);
      robot.setActivelyShooting(true);
    } else if (joystick.getRawButton(14)) {
      robot.setShooterPowerPct(-0.2, -0.2);
      robot.setActivelyShooting(false);
    } else {
      robot.setShooterPowerPct(0.0, 0.0);
      robot.setActivelyShooting(false);

    }

    if (joystick.getRawButton(7)) robot.raiseCollector();
    if (joystick.getRawButton(8)) robot.lowerCollector();

    if (joystick.getRawButton(6)) robot.turnOnPTO();
    if (joystick.getRawButton(9)) robot.turnOffPTO();

    if (joystick.getRawButton(5)) robot.setArmsForward();
    if (joystick.getRawButton(10)) robot.setArmsBackward();

    if (joystick.getRawButtonPressed(4)){
      countHang = (countHang + 1) % 2;
    }

    if (countHang == 1) {
      hang = true;
    } else {
      hang = false;
    }

    if (hang){
      if (reset) {
        testHang.begin(robot);
        reset = false;
      }
      testHang.step(robot);
    }
    else{
      reset = true;
    }


  }

  public enum POVDirection {
    NORTH(0),
    NORTHEAST(45),
    EAST(90),
    SOUTHEAST(135),
    SOUTH(180),
    SOUTHWEST(225),
    WEST(270), //best
    NORTHWEST(315),
    NULL(-1);

    private final int angle;

    POVDirection(int angle) {
      this.angle = angle;
    }

    public int getAngle() {
      return angle;
    }

    //Kevin voodoo to turn ints into directions
    public static final Map<Integer, POVDirection> directionMap =
            Arrays.stream(POVDirection.values()).collect(
                    Collectors.toMap(
                            POVDirection::getAngle,
                            Function.identity()
                    )
            );

    public static POVDirection getDirection(int angle) {
      return directionMap.getOrDefault(angle, POVDirection.NULL);
    }
  }
}
