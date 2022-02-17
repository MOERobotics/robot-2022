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
import frc.robot.generic.Falcon;
import frc.robot.generic.GenericRobot;
import frc.robot.generic.Lightning;

public class Robot extends TimedRobot {

  GenericRobot robot = new Falcon();
  Joystick joystick = new Joystick(0);
  GenericAutonomous autonomous = new BallCtoTerminal();


  int averageTurretXSize = 2;
  double[] averageX = new double [averageTurretXSize];

  double turretx;
  double turrety;
  double turretarea;
  double turretv;
  int counter = 0;
  double average;
  double currentTurretPower;


  PIDController turretPIDController;



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

    SmartDashboard.putBoolean("Have upper cargo?", robot.getUpperCargo());
    SmartDashboard.putBoolean("Have lower cargo?", robot.getLowerCargo());

    //SmartDashboard.putBoolean("Has detected cargo?", robot.hasFoundCargo());

    SmartDashboard.getNumber("Collector intake power", robot.getCollectorIntakePercentage());
    //SmartDashboard.getBoolean("Sees target?", robot.isTargetFound());

    SmartDashboard.getNumber("Vision target x", robot.getTargetX());
    SmartDashboard.getNumber("Vision target y", robot.getTargetY());
    SmartDashboard.getNumber("Vision target angle", robot.getTargetAngle());
    SmartDashboard.getNumber("Vision target dist", robot.getTargetDistance());

    SmartDashboard.getNumber("Turret direction angle ticks", robot.getTurretAngle());
    SmartDashboard.getNumber("Turret direction angle degrees", robot.getTurretAngleDegrees());

    SmartDashboard.getNumber("Turret direction motor pct", robot.getTurretPowerPct());

    SmartDashboard.getNumber("Turret pitch angle", robot.getTurretPitchAngle());
    SmartDashboard.getNumber("Turret pitch motor pct", robot.getTurretPitchPowerPct());

    SmartDashboard.getNumber("Shooter top motor pct", robot.getShooterPowerPctTop());
    SmartDashboard.getNumber("Shooter bottom motor pct", robot.getShooterPowerPctBottom());

    SmartDashboard.getNumber("Shooter top motor rpm", robot.getShooterRPMTop());
    SmartDashboard.getNumber("Shooter bottom motor rpm", robot.getShooterRPMBottom());

    SmartDashboard.getNumber("Shooter calculate distance", robot.getShooterTargetDistance());
    SmartDashboard.getNumber("Shooter calculate height", robot.getShooterTargetHeight());

    SmartDashboard.putNumber("Joystick raw X", joystick.getX());
    SmartDashboard.putNumber("Joystick raw Y", joystick.getY());

    SmartDashboard.putNumber("Autonomous Step", autonomous.autonomousStep);

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

  @Override public void teleopPeriodic() {
    double jx =  joystick.getX();
    double jy = -joystick.getY();

    //joystick deaden: yeet smol/weird joystick values when joystick is at rest
    double cutoff = 0.05;
    if(jy > -cutoff && jy < cutoff) jy = 0;
    if(jx > -cutoff && jx < cutoff) jx = 0;

    //moved this to after joystick deaden because deaden should be focused on the raw joystick values
    double scaleFactor = 1.0;

    robot.drivePercent(
        (jy+jx) * scaleFactor,
        (jy-jx) * scaleFactor
    );

    //note to self: buttons currently assume mirrored joystick setting
    if      (joystick.getRawButton(11)) robot.setCollectorIntakePercentage( 1.0);
    else if (joystick.getRawButton(16)) robot.setCollectorIntakePercentage(-1.0);
    else                                robot.setCollectorIntakePercentage( 1.0);

    if      (joystick.getRawButton(12)) robot.setTurretPowerPct( 0.2);
    else if (joystick.getRawButton(15)) robot.setTurretPowerPct(-0.2);
    else                                robot.setTurretPowerPct( 0.0);

    if      (joystick.getRawButton(13)) robot.setShooterPowerPct( 0.2,  0.2);
    else if (joystick.getRawButton(14)) robot.setShooterPowerPct(-0.2, -0.2);
    else                                robot.setShooterPowerPct( 0.0,  0.0);

    if      (joystick.getRawButton( 7)) robot.raiseCollector();
    if      (joystick.getRawButton( 8)) robot.lowerCollector();

    if      (joystick.getRawButton( 6)) robot.turnOnPTO();
    if      (joystick.getRawButton( 9)) robot.turnOffPTO();

    if      (joystick.getRawButton( 5)) robot.setArmsForward();
    if      (joystick.getRawButton(10)) robot.setArmsBackward();


    //Start of Daniel+Saiarun Turret test
    average = 0;

    if(robot.isTargetFound()) {
      counter = counter % averageTurretXSize;
      averageX[counter] = robot.getTargetX();
      counter++;
    }
    average = 0;
    for(double i: averageX){
      average += i;
    }
    average /= averageTurretXSize;
    SmartDashboard.putNumber("Average", average);

    if (joystick.getRawButtonPressed(1)) turretPIDController.reset();
    if (joystick.getRawButton(1) && turretv !=0){
      currentTurretPower = turretPIDController.calculate(average);
    } else {
      if      (joystick.getRawButton(3))  currentTurretPower = -0.1;
      else if (joystick.getRawButton(4))  currentTurretPower =  0.1;
      else                                currentTurretPower =  0.0;
    }

    robot.setTurretPowerPct(currentTurretPower);

  }

  @Override public void disabledInit() {}

  @Override public void disabledPeriodic() {
    if (joystick.getRawButton(1)){
      robot.resetAttitude();
      robot.resetEncoders();
    }

    if (joystick.getRawButton(4)){
      autonomous = new autoArc();

    }
    if (joystick.getRawButton(5)){
      autonomous = new BallAtoTerminal();
    }
    if (joystick.getRawButton(6)){
      autonomous = new BallBtoTerminal();
    }
    if (joystick.getRawButton(7)){
      autonomous = new BallCtoTerminal();
    }

  }

  @Override public void testInit() {}

  @Override public void testPeriodic() {}
}