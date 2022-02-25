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
import frc.robot.generic.*;

import java.util.Arrays;
import java.util.Map;
import java.util.function.Function;
import java.util.stream.Collectors;

import static frc.robot.Robot.ArmsState.*;
import static frc.robot.Robot.CollectorState.*;
import static frc.robot.Robot.ClimberState.*;
import static frc.robot.Robot.ShooterState.*;
import static frc.robot.Robot.TurretState.*;

public class Robot extends TimedRobot {

  //Instantiate autonomous once, don't create unnecessary duplicates
  //Add new Autos here when they're authored
  public static final GenericAutonomous
      autoArc         = new autoArc(),
      simpleATerminal = new SimpleATerminal(),
      simpleBTerminal = new SimpleBTerminal(),
      simpleCTerminal = new SimpleCTerminal();

  GenericRobot robot = new Lightning();
  Joystick joystick = new Joystick(0);
  Joystick xbox = new Joystick(1);
  GenericAutonomous autonomous = autoArc;


  int averageTurretXSize = 2;
  double[] averageX = new double [averageTurretXSize];

  double turretx;
  double turrety;
  double turretarea;
  double turretv;
  int counter = 0;
  double average;
  double currentTurretPower;

  double turretPitch = 0;
  int targetRPM = 4000;

  PIDController turretPIDController;

  TurretState        turretState = TURRET_MANUAL;
  ClimberState      climberState = CLIMBER_INACTIVE;
  ShooterState      shooterState = SHOOTER_INACTIVE;
  CollectorState  collectorState = COLLECTOR_INACTIVE;
  ArmsState            armsState = ARMS_UP;



  @Override public void robotInit() {}

  @Override public void robotPeriodic() {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");

    //read values periodically
    turretx = tx.getDouble(0.0);
    turrety = ty.getDouble(0.0);
    turretarea = ta.getDouble(0.0);
    turretv = tv.getDouble(0.0);

    SmartDashboard.putNumber("tv", turretv);

    SmartDashboard.putNumber("LimelightX", turretx);
    SmartDashboard.putNumber("LimelightY", turrety);
    SmartDashboard.putNumber("LimelightArea", turretarea);

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

    SmartDashboard.putNumber("Turret pitch angle", robot.getTurretPitchAngle());
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


    SmartDashboard.putNumber("Target RPM", targetRPM);
    SmartDashboard.putNumber("Turret Hood Pitch", turretPitch);

    SmartDashboard.putStringArray("State", new String[]{
        climberState.name(),
        shooterState.name(),
        turretState.name(),
        collectorState.name(),
        armsState.name()
    });
    //SmartDashboard.putStringArray("State", new String[]{
    SmartDashboard.putString("climberState", climberState.name());
    SmartDashboard.putString("shooterState", shooterState.name());
    SmartDashboard.putString("turretState", turretState.name());
    SmartDashboard.putString("collectorState", collectorState.name());
    SmartDashboard.putString("armsState", armsState.name());
    //});

  }

  @Override public void autonomousInit() {
    autonomous.autonomousInit(robot);
  }

  @Override public void autonomousPeriodic() {
    autonomous.autonomousPeriodic(robot);
  }

  @Override public void teleopInit() {
      turretPIDController = new PIDController(robot.turretPIDgetP(), robot.turretPIDgetI(), robot.turretPIDgetD());
  }



  public static final double deadzone = 0.15;
  @Override public void teleopPeriodic() {
    double
        driveLPower = 0,
        driveRPower = 0,
        turretPower = 0,
        collectorPower = 0,
        indexerPower = 0;

    /**
     * Miscellanea
     */

    switch (POVDirection.getDirection(xbox.getPOV())) {
      case NORTH: targetRPM = 3250; turretPitch = 0.00; break;
      case EAST : targetRPM = 4000; turretPitch = 0.00; break;
      case SOUTH: targetRPM = 5000; turretPitch = 0.00; break;
      case WEST : targetRPM = 5500; turretPitch = 0.00; break;
    }

    //Fine Tune turret hood position
    double rJoyRY = xbox.getRawAxis( 5);
    if (rJoyRY > -deadzone && rJoyRY < deadzone) rJoyRY = 0;
    turretPitch += rJoyRY * 0.05;
    if (turretPitch < -1) turretPitch = -1;
    if (turretPitch >  1) turretPitch =  1;
    robot.setTurretPitchPowerPct(turretPitch);

    //Fine Tune shooter speed
    double rJoyLY = xbox.getRawAxis( 1);
    if (rJoyLY > -deadzone && rJoyLY < deadzone) rJoyLY = 0;
    targetRPM += rJoyLY * 0.5;
    if (targetRPM <  250) targetRPM =  250;
    if (targetRPM > 6000) targetRPM = 6000;


    if (joystick.getRawButton( 2)) robot.raiseCollector();
    else robot.lowerCollector();




    /**
     * Drive/Climb
     */

    if (joystick.getRawButton(13)) climberState = CLIMBER_MANUAL;
    if (    xbox.getRawButton( 7)) climberState = CLIMBER_ACTIVE;
    if (    xbox.getRawButton( 8)) climberState = CLIMBER_INACTIVE;
    if (joystick.getRawButton(14)) climberState = CLIMBER_INACTIVE;

    switch (climberState) {

      case CLIMBER_MANUAL:
        robot.turnOnPTO();
        if (joystick.getRawButton( 6)) driveLPower =  0.5;
        if (joystick.getRawButton( 9)) driveLPower = -0.5;
        if (joystick.getRawButton( 7)) driveRPower =  0.5;
        if (joystick.getRawButton( 8)) driveRPower = -0.5;

        if      (joystick.getRawButton( 5)) armsState = ARMS_UP;
        else if (joystick.getRawButton(10)) armsState = ARMS_BACK;

        break;

      case CLIMBER_ACTIVE:
        robot.turnOnPTO();
        //Do cool things
        break;

      default:
      case CLIMBER_INACTIVE:
        robot.turnOffPTO();
        armsState = ARMS_UP;

        double
            lJoyY = -joystick.getY(),
            lJoyX =  joystick.getX();

        if (lJoyY > -deadzone && lJoyY < deadzone) lJoyY = 0;
        if (lJoyX > -deadzone && lJoyX < deadzone) lJoyX = 0;

        double scaleFactor = 1.0;
        lJoyY *= scaleFactor;
        lJoyX *= scaleFactor;

        driveLPower = lJoyY + lJoyX;
        driveRPower = lJoyY - lJoyX;

        if (joystick.getRawButton(11)) {driveLPower =  0.30; driveRPower =  0.30;}
        if (joystick.getRawButton(16)) {driveLPower = -0.30; driveRPower = -0.30;}
        if (joystick.getRawButton( 4)) {driveLPower =  0.30; driveRPower = -0.30;}
        if (joystick.getRawButton( 3)) {driveLPower = -0.30; driveRPower =  0.30;}
        break;

    } //switch (climberState)

    /**
     * Turret
     */
    if (xbox.getRawAxis(4) > 0.80) turretState = TURRET_ACTIVE;
    else                           turretState = TURRET_MANUAL;

    switch (turretState) {
      default:
      case TURRET_MANUAL:
        if (xbox.getRawButton( 5)) turretPower = -0.45;
        if (xbox.getRawButton( 6)) turretPower =  0.45;
        //Leave the tracking array empty for the next autotrack
        turretPIDController.reset();
        for (int i = 0; i < averageTurretXSize; i++)
          averageX[i] = 0;
        break;

      case TURRET_ACTIVE:
        //If I can't see, don't move
        if(!robot.isTargetFound()) break;

        counter = (counter + 1) % averageTurretXSize;
        averageX[counter] = robot.getTargetX();
        turretPIDController.calculate(average);

        double average = 0;
        for (int i = 0; i < averageTurretXSize; i++)
          average += averageX[i];
        average /= averageTurretXSize;
        turretPower = turretPIDController.calculate(average);
        break;
    } //switch (turretState)

    /**
     * Collector
     */

    collectorState = COLLECTOR_INACTIVE;
    if (joystick.getRawButton( 1)) collectorState = COLLECTOR_COLLECTING;
    if (    xbox.getRawButton( 2)) collectorState = COLLECTOR_EJECTING;
    if (    xbox.getRawButton( 4)) collectorState = COLLECTOR_COLLECTING;

    switch (collectorState) {
      case COLLECTOR_COLLECTING:
        if (
            !robot.getUpperCargo()
        ) {
          indexerPower = 1.00;
          collectorPower = 1.00;
        } else if (
            !robot.getLowerCargo()
        ) {
          collectorPower = 1.00;
        }
        break;

      case COLLECTOR_EJECTING:
        indexerPower = -1.00;
        collectorPower = -1.00;
        break;

      default:
      case COLLECTOR_INACTIVE:
        break;
    } // switch (collectorState)


    /**
     * Shooter
     */

    if (    xbox.getRawButton( 3)) shooterState = SHOOTER_ACTIVE;
    else                           shooterState = SHOOTER_INACTIVE;

    switch (shooterState) {
      case SHOOTER_ACTIVE:
        robot.setActivelyShooting(true);
        robot.setShooterRPM(targetRPM,targetRPM);
        if (
            robot.isReadyToShoot() &&
            xbox.getRawButton( 1)
        ) {
          indexerPower = 1.00;
        }
        break;

      default:
      case SHOOTER_INACTIVE:
        robot.setActivelyShooting(false);
        robot.setShooterPowerPct(0,0);
        break;
    } //switch (shooterState)

    /**
     * Arms
     */

    switch (armsState) {
      case ARMS_BACK:
        robot.setArmsBackward();
        break;
      case ARMS_UP:
        robot.setArmsForward();
        break;
      default:
      case ARMS_UNKNOWN:
        break;
    }

    /**
     * Safety
     */

    if (
        turretPower < 0 && //todo: degrees
        robot.getAlternateTurretAngle() < 30
    ) turretPower = 0;

    if (
        turretPower > 0 &&
        robot.getAlternateTurretAngle() > 340
    ) turretPower = 0;


    /**
     * Send powers
     */


    robot.drivePercent(
        driveLPower,
        driveRPower
    );
    robot.setTurretPowerPct(turretPower);
    robot.setCollectorIntakePercentage(collectorPower);
    robot.setIndexerIntakePercentage(indexerPower);

  }

  @Override public void disabledInit() {}

  @Override public void disabledPeriodic() {
    if (joystick.getRawButton(1)){
      robot.resetAttitude();
      robot.resetEncoders();
    }

    if (joystick.getRawButton(4)) autonomous = autoArc;
    if (joystick.getRawButton(5)) autonomous = simpleATerminal;
    if (joystick.getRawButton(6)) autonomous = simpleBTerminal;
    if (joystick.getRawButton(7)) autonomous = simpleCTerminal;

  }

  @Override public void testInit() {}

  @Override public void testPeriodic() {

    if (xbox.getRawButton(1)){
      robot.setTurretPitchPowerPct(1);
    }
    else if (xbox.getRawButton(2)){
      robot.setTurretPitchPowerPct(-1);
    }
    else{
      robot.setTurretPitchPowerPct(0);
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


    //currently Jack has no clue what axises these are supposed to be
    int leftAxis = 1; int rightAxis = 5;
    double tolerance = 0.8;
    double drivePower = 0.2;

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
    } else {

      double driveX =  joystick.getX();
      double driveY = -joystick.getY();

      //joystick deaden: yeet smol/weird joystick values when joystick is at rest
      double cutoff = 0.05;
      if(driveX > -cutoff && driveX < cutoff) driveX = 0;
      if(driveY > -cutoff && driveY < cutoff) driveY = 0;

      //moved this to after joystick deaden because deaden should be focused on the raw joystick values
      double scaleFactor = 1.0;

      driveLeft = (driveY+driveX) * scaleFactor;
      driveRight = (driveY-driveX) * scaleFactor;
    }
    robot.drivePercent(driveLeft, driveRight);
  }


  public static enum CollectorState {
    COLLECTOR_COLLECTING,
    COLLECTOR_EJECTING,
    COLLECTOR_INACTIVE
  }

  public static enum ShooterState {
    SHOOTER_ACTIVE,
    SHOOTER_INACTIVE
  }

  public static enum ClimberState {
    CLIMBER_ACTIVE,
    CLIMBER_INACTIVE,
    CLIMBER_MANUAL
  }

  public static enum TurretState {
    TURRET_ACTIVE,
    TURRET_MANUAL
  }

  public static enum ArmsState {
    ARMS_UP,
    ARMS_BACK,
    ARMS_UNKNOWN
  }

  public enum POVDirection {
    NORTH     (   0),
    NORTHEAST (  45),
    EAST      (  90),
    SOUTHEAST ( 135),
    SOUTH     ( 180),
    SOUTHWEST ( 225),
    WEST      ( 270), //best
    NORTHWEST ( 315),
    NULL      (  -1);

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