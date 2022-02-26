// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.*;
import frc.robot.autonomous.GenericAutonomous;
import frc.robot.command.*;
import frc.robot.generic.GenericRobot;
import frc.robot.generic.Lightning;
import frc.robot.generic.TurretBot;

public class Robot extends TimedRobot {

  //Instantiate autonomous once, don't create unnecessary duplicates
  //Add new Autos here when they're authored
  public static final GenericAutonomous
      autoArc         = new autoArc(),
      simpleATerminal = new BallAtoTerminal(),
      simpleBTerminal = new BallBtoTerminal(),
      simpleCTerminal = new BallCtoTerminal(),
      CTerminalReturn = new BallCtoTerminalReturn();

  GenericRobot robot = new Lightning();
  Joystick joystick = new Joystick(0);
  GenericCommand command = new Hang();
  Joystick xbox = new Joystick(1);
  GenericAutonomous autonomous = autoArc;


  int averageTurretXSize = 2;
  double[] averageX = new double [averageTurretXSize];

  /*double turretx;
  double turrety;
  double turretarea;
  double turretv;*/
  int counter = 0;
  double average;
  double currentTurretPower;
  boolean hang = false;
  int count = 0;
  boolean reset = true;
  double maxCurrentLeftA = 0;
  double maxCurrentLeftB = 0;
  double maxCurrentRightA = 0;
  double maxCurrentRightB = 0;
  boolean ActuallyHanging = false;




  PIDController turretPIDController;



  @Override public void robotInit() {}

  @Override public void robotPeriodic() {

    if (robot.getLeftACurrent() > maxCurrentLeftA){
      maxCurrentLeftA = robot.getLeftACurrent();
    }
    if (robot.getLeftBCurrent() > maxCurrentLeftB){
      maxCurrentLeftB = robot.getLeftBCurrent();
    }
    if(robot.getRightACurrent() > maxCurrentRightA){
      maxCurrentRightA = robot.getRightACurrent();
    }
    if (robot.getRightBCurrent() > maxCurrentRightB){
      maxCurrentRightB = robot.getRightBCurrent();
    }

   /* NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");

//read values periodically
    turretx = tx.getDouble(0.0);
    turrety = ty.getDouble(0.0);
    turretarea = ta.getDouble(0.0);
    turretv = tv.getDouble(0.0);
*/
    SmartDashboard.putBoolean("tv", robot.isTargetFound());
    SmartDashboard.putNumber("LeftACurrentMax", maxCurrentLeftA);
    SmartDashboard.putNumber("LeftBCurrentMax", maxCurrentLeftB);
    SmartDashboard.putNumber("RightACurrentMax", maxCurrentRightA);
    SmartDashboard.putNumber("RightBCurrentMax", maxCurrentRightB);

    SmartDashboard.putNumber("LimelightX", robot.getTargetX());
    SmartDashboard.putNumber("LimelightY", robot.getTargetY());
    SmartDashboard.putNumber("LimelightArea", robot.getTargetArea());

    SmartDashboard.putNumber("Drive left pct", robot.getDriveLeftPercentage());
    SmartDashboard.putNumber("Drive right pct", robot.getDriveRightPercentage());
    SmartDashboard.putNumber("Drive left rpm", robot.getDriveLeftRPM());
    SmartDashboard.putNumber("Drive right rpm", robot.getDriveRightRPM());

    SmartDashboard.putNumber("Left encoder Ticks", robot.encoderTicksLeftDrive());
    SmartDashboard.putNumber("Right encoder Ticks", robot.encoderTicksRightDrive());

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

    SmartDashboard.putNumber("Vision target x", robot.getTargetX());
    SmartDashboard.putNumber("Vision target y", robot.getTargetY());
    SmartDashboard.putNumber("Vision target angle", robot.getTargetAngle());
    SmartDashboard.putNumber("Vision target dist", robot.getTargetDistance());

    SmartDashboard.putNumber("Turret direction angle ticks", robot.getTurretAngle());
    SmartDashboard.putNumber("Turret direction angle degrees", robot.getTurretAngleDegrees());
    SmartDashboard.putNumber("Alternate turret angle ticks", robot.getAlternateTurretAngle());

    SmartDashboard.putNumber("Turret direction motor pct", robot.getTurretPowerPct());

    SmartDashboard.putNumber("Turret direction motor pct", robot.getTurretPowerPct());

    SmartDashboard.putNumber("Turret pitch position", robot.getTurretPitchPosition());
    SmartDashboard.putNumber("Turret pitch motor pct", robot.getTurretPitchPowerPct());

    SmartDashboard.putNumber("Shooter top motor pct", robot.getShooterPowerPctTop());
    SmartDashboard.putNumber("Shooter bottom motor pct", robot.getShooterPowerPctBottom());

    SmartDashboard.putNumber("Shooter top motor rpm", robot.getShooterRPMTop());
    SmartDashboard.putNumber("Shooter bottom motor rpm", robot.getShooterRPMBottom());

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

    SmartDashboard.putNumber("leftEncoderRaw", robot.encoderTicksLeftDrive());
    SmartDashboard.putNumber("rightEncoderRaw", robot.encoderTicksRightDrive());
    SmartDashboard.putBoolean("leftTapeSensor", robot.getFloorSensorLeft());
    SmartDashboard.putBoolean("rightTapeSensor", robot.getFloorSensorRight());
    SmartDashboard.putBoolean("leftCLimberSensor", robot.getClimbSensorLeft());
    SmartDashboard.putBoolean("rightClimberSensor", robot.getClimbSensorRight());

    SmartDashboard.putNumber("rightArmEncoder", robot.armHeightRight());
    SmartDashboard.putNumber("leftArmEncoder", robot.armHeightLeft());

    SmartDashboard.putBoolean("hang", hang);
    SmartDashboard.putNumber("count", count);

    SmartDashboard.putNumber("CommandStep", command.commandStep);


  }

  @Override public void autonomousInit() {
    autonomous.autonomousInit(robot);
  }

  @Override public void autonomousPeriodic() {
    autonomous.autonomousPeriodic(robot);
  }

  @Override public void teleopInit() {
      turretPIDController = new PIDController(robot.turretPIDgetP(), robot.turretPIDgetI(), robot.turretPIDgetD());
      hang = false;
      count = 0;
  }

  @Override public void teleopPeriodic() {

    //note to self: buttons currently assume mirrored joystick setting
    if (joystick.getRawButtonPressed(8)) {
      count = (count + 1) % 2;
    }
    if (count == 1) {
      hang = true;
    } else {
      hang = false;
    }

    if (!hang) {
      reset = true;

      double jx = joystick.getX();
      double jy = -joystick.getY();

      //joystick deaden: yeet smol/weird joystick values when joystick is at rest
      double cutoff = 0.05;
      if (jy > -cutoff && jy < cutoff) jy = 0;
      if (jx > -cutoff && jx < cutoff) jx = 0;

      //moved this to after joystick deaden because deaden should be focused on the raw joystick values
      double scaleFactor = 1.0;

      //robot PTO not on arms, give joystick carte blanche
      if (!robot.getPTOState()) {
        robot.drivePercent(
                (jy + jx) * scaleFactor,
                (jy - jx) * scaleFactor
        );
      }


      SmartDashboard.putNumber("XBOX AXIS DEBUG - 0 ", xbox.getRawAxis(0));
      SmartDashboard.putNumber("XBOX AXIS DEBUG - 1 ", xbox.getRawAxis(1));
      SmartDashboard.putNumber("XBOX AXIS DEBUG - 2 ", xbox.getRawAxis(2));
      SmartDashboard.putNumber("XBOX AXIS DEBUG - 3 ", xbox.getRawAxis(3));


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
          driveLeft = drivePower;
        } else if (xbox.getRawAxis(leftAxis) < -tolerance) {
          driveLeft = -drivePower;
        }
        if (robot.getPTOState()) {
          if (xbox.getRawAxis(leftAxis) > tolerance) {
            driveLeft = drivePower;
          } else if (xbox.getRawAxis(leftAxis) < -tolerance) {
            driveLeft = -drivePower;
          }

          if (xbox.getRawAxis(rightAxis) > tolerance) {
            driveRight = drivePower;
          } else if (xbox.getRawAxis(rightAxis) < -tolerance) {
            driveRight = -drivePower;
          }
          robot.drivePercent(driveLeft, driveRight);
        }

      }
      if (hang) {
        ActuallyHanging = true;
        SmartDashboard.putBoolean("we really are hanging", ActuallyHanging);
        if (reset) {
          command.begin(robot);
          reset = false;
        }
        command.step(robot);
      }


      //Start of Daniel+Saiarun Turret test
      average = 0;

      if (robot.isTargetFound()) {
        counter = counter % averageTurretXSize;
        averageX[counter] = robot.getTargetX();
        counter++;
      }
      average = 0;
      for (double i : averageX) {
        average += i;
      }
      average /= averageTurretXSize;
      SmartDashboard.putNumber("Average", average);

      if (joystick.getRawButtonPressed(1)) turretPIDController.reset();
      if (joystick.getRawButton(1) && robot.isTargetFound()) {
        currentTurretPower = turretPIDController.calculate(average);
      } else {
        if (joystick.getRawButton(3)) currentTurretPower = -0.1;
        else if (joystick.getRawButton(4)) currentTurretPower = 0.1;
        else currentTurretPower = 0.0;
      }

      if (joystick.getRawButton(12)) currentTurretPower = 0.2;
      else if (joystick.getRawButton(15)) currentTurretPower = -0.2;

      robot.setTurretPowerPct(currentTurretPower);

      double shooterTargetRPM = 0;
      if (joystick.getRawButton(11)) {
        shooterTargetRPM = robot.getShooterTargetRPM();
      } else {
        shooterTargetRPM = 0;
      }

      robot.setShooterRPM(shooterTargetRPM, shooterTargetRPM);

      if (joystick.getRawButton(16)) {
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


    double pitchChange = 0;
    if (joystick.getRawButtonPressed(13)){
      pitchChange = 0.02;
    }
    else if (joystick.getRawButtonPressed(14)){
      pitchChange = -0.02;
    }
    else{
      pitchChange = 0;
    }
    double newPos = robot.getTurretPitchPosition() + pitchChange;

      robot.setTurretPitchPosition(newPos);


      //Collector indexer logic based on cargo already in sensors (from jack)
      double defCollectorPower = 1;
      double defIndexerPower = 1;
      double curCollector;
      double curIndexer;

    //button 2 = bottom center button
    if(joystick.getRawButton(2)){
      if(!robot.getUpperCargo()){
        curCollector = defCollectorPower;
        curIndexer = defIndexerPower;
      }
      else{
        curIndexer = 0;
        if(!robot.getLowerCargo()){
          curCollector = defCollectorPower;
        }
        else{
          curCollector = 0;
        }
      }
      if(robot.isActivelyShooting()){
        curIndexer = defIndexerPower;
      }
    }
    else if (joystick.getRawButton(5)){
      curCollector = -defCollectorPower;
      curIndexer = -defIndexerPower;
    }
    else{
      curCollector = 0;
      curIndexer = 0;
    }

      //Cargo is on upper sensor and we want to yeet it: indexer needs to push it past sensor
      if (robot.isActivelyShooting() && robot.getUpperCargo()) {
        curIndexer = defIndexerPower;
      }

      robot.setCollectorIntakePercentage(curCollector);
      robot.setIndexerIntakePercentage(curIndexer);

    //robot.drivePercent(0, 0);

    }
  }

  @Override public void disabledInit() {}

  @Override public void disabledPeriodic() {
    hang = false;
    count = 0;
    if (joystick.getRawButton(1)){
      robot.resetAttitude();
      robot.resetEncoders();
    }

    if (joystick.getRawButton(4)) autonomous = autoArc;
    if (joystick.getRawButton(5)) autonomous = simpleATerminal;
    if (joystick.getRawButton(6)) autonomous = simpleBTerminal;
    if (joystick.getRawButton(7)) autonomous = simpleCTerminal;
    if (joystick.getRawButton(9)) autonomous = CTerminalReturn;

  }

  @Override public void testInit() {}

  @Override public void testPeriodic() {

      double pitchChange = 0;
    if (xbox.getRawButton(1)){
      pitchChange = 0.02;
    }
    else if (xbox.getRawButton(2)){
      pitchChange = -0.02;
    }
    else{
      pitchChange = 0;
    }
      double newPos = robot.getTurretPitchPosition() + pitchChange;
      if(newPos < 0) newPos = 0;
      if(newPos > 1) newPos = 1;

      robot.setTurretPitchPosition(newPos);

    double driveX =  joystick.getX();
    double driveY = -joystick.getY();

    //joystick deaden: yeet smol/weird joystick values when joystick is at rest
    double cutoff = 0.05;
    if(driveX > -cutoff && driveX < cutoff) driveX = 0;
    if(driveY > -cutoff && driveY < cutoff) driveY = 0;

    //moved this to after joystick deaden because deaden should be focused on the raw joystick values
    double scaleFactor = 1.0;

    if(!robot.getPTOState()){
      robot.drivePercent(
              (driveY+driveX) * scaleFactor,
              (driveY-driveX) * scaleFactor
      );
    }

    int leftAxis = 1; int rightAxis = 5;
    double tolerance = 0.8;
    double drivePower = 0.2;

    if      (joystick.getRawButton(12)) robot.setTurretPowerPct( 0.2);
    else if (joystick.getRawButton(15)) robot.setTurretPowerPct(-0.2);
    else                                robot.setTurretPowerPct( 0.0);

    double driveLeft = 0;
    double driveRight = 0;

    if(robot.getPTOState()){
      if(xbox.getRawAxis(leftAxis) > tolerance){
        driveLeft = drivePower;
      }
      else if(xbox.getRawAxis(leftAxis) < -tolerance){
        driveLeft = -drivePower;
      }

      if(xbox.getRawAxis(rightAxis) > tolerance){
        driveRight = drivePower;
      }
      else if(xbox.getRawAxis(rightAxis) < -tolerance){
        driveRight = -drivePower;
      }
      robot.drivePercent(driveLeft, driveRight);
    }

    //note to self: buttons control mirrored joystick setting
    if(joystick.getRawButton(11)) {
      robot.setCollectorIntakePercentage(1);
      robot.setIndexerIntakePercentage(1);
    }
    else if(joystick.getRawButton(16)){
      robot.setCollectorIntakePercentage(-1);
      robot.setIndexerIntakePercentage(-1);
    }
    else{
      robot.setCollectorIntakePercentage(0);
      robot.setIndexerIntakePercentage(0);
    }

    if      (joystick.getRawButton(12)) robot.setTurretPowerPct( 0.2);
    else if (joystick.getRawButton(15)) robot.setTurretPowerPct(-0.2);
    else                                robot.setTurretPowerPct( 0.0);


    if      (joystick.getRawButton(13)){
      robot.setShooterPowerPct( 0.2,  0.2);
      robot.setActivelyShooting(true);
    }
    else if (joystick.getRawButton(14)){
      robot.setShooterPowerPct(-0.2, -0.2);
      robot.setActivelyShooting(false);
    }
    else                               {
      robot.setShooterPowerPct( 0.0,  0.0);
      robot.setActivelyShooting(false);

    }

    if      (joystick.getRawButton( 7)) robot.raiseCollector();
    if      (joystick.getRawButton( 8)) robot.lowerCollector();

    if      (joystick.getRawButton( 6)) robot.turnOnPTO();
    if      (joystick.getRawButton( 9)) robot.turnOffPTO();

    if      (joystick.getRawButton( 5)) robot.setArmsForward();
    if      (joystick.getRawButton(10)) robot.setArmsBackward();

  }
}