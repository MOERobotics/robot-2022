// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.GenericRobot;
import frc.robot.generic.Lightning;

public class Robot extends TimedRobot {

  GenericRobot robot = new Lightning();
  Joystick joystick = new Joystick(0);

  double[] averageTurretX = new double [6];

  double turretx;
  double turrety;
  double turretarea;


  @Override public void robotInit() {}

  @Override public void robotPeriodic() {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

//read values periodically
    turretx = tx.getDouble(0.0);
    turrety = ty.getDouble(0.0);
    turretarea = ta.getDouble(0.0);

    SmartDashboard.putNumber("LimelightX", turretx);
    SmartDashboard.putNumber("LimelightY", turrety);
    SmartDashboard.putNumber("LimelightArea", turretarea);

    SmartDashboard.putNumber("Drive left pct", robot.getDriveLeftPercentage());
    SmartDashboard.putNumber("Drive right pct", robot.getDriveRightPercentage());
    SmartDashboard.putNumber("Drive left rpm", robot.getDriveLeftRPM());
    SmartDashboard.putNumber("Drive right rpm", robot.getDriveRightRPM());

    SmartDashboard.putNumber("Left encoder", robot.getDriveDistanceInchesLeft());
    SmartDashboard.putNumber("Right encoder", robot.getDriveDistanceInchesRight());

    SmartDashboard.putNumber("Yee", robot.getYee());
    SmartDashboard.putNumber("Pitch", robot.getPitch());
    SmartDashboard.putNumber("Rollll", robot.getRoll());
    SmartDashboard.putNumber("Linear speed", robot.getLinearVelocity());

    SmartDashboard.putBoolean("Have upper cargo?", robot.getUpperCargo());
    SmartDashboard.putBoolean("Have lower cargo?", robot.getLowerCargo());

    SmartDashboard.putNumber("Cargo inventory", robot.getCargoCount());
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

  }

  @Override public void autonomousInit() {}

  @Override public void autonomousPeriodic() {}

  @Override public void teleopInit() {


  }

  @Override public void teleopPeriodic() {
    double jx = joystick.getX();
    double jy = -joystick.getY();

    //joystick deaden: yeet smol/weird joystick values when joystick is at rest
    double cutoff = 0.05;
    if(jy > -cutoff && jy < cutoff) jy = 0;
    if(jx > -cutoff && jx < cutoff) jx = 0;

    //moved this to after joystick deaden because deaden should be focused on the raw joystick values
    double scaleFactor = 1.0;
    jx *= scaleFactor;
    jy *= scaleFactor;

    robot.drivePercent(jy+jx,jy-jx);

    /* if(joystick.getRawButton(1)){
      robot.setShooterPowerPct(0.2, 0.2);
    }*/


    //note to self: buttons control mirrored joystick setting
    if(joystick.getRawButton(11)) robot.setCollectorIntakePercentage(0.6);
    else if(joystick.getRawButton(16)) robot.setCollectorIntakePercentage(-0.6);
    else robot.setCollectorIntakePercentage(0);

    if(joystick.getRawButton
            (12)) robot.setTurretPowerPct(0.2);
    else if(joystick.getRawButton(15)) robot.setTurretPowerPct(-0.2);
    else robot.setTurretPowerPct(0);

    if(joystick.getRawButton(13)) robot.setShooterPowerPct(0.2, 0.2);
    else if(joystick.getRawButton(14)) robot.setShooterPowerPct(-0.2, -0.2);
    else robot.setShooterPowerPct(0, 0);

    if(joystick.getRawButton(7)) robot.raiseCollector();
    if(joystick.getRawButton(8)) robot.lowerCollector();

    if(joystick.getRawButton(6)) robot.turnOnPTO();
    if(joystick.getRawButton(9)) robot.turnOffPTO();

    if(joystick.getRawButton(5)) robot.setArmsForward();
    if(joystick.getRawButton(10)) robot.setArmsBackward();


    //Start of Daniel+Saiarun Turret test

    int counter = 0;
    double average = 0;

    averageTurretX[counter%6] = turretx;
    counter++;
    for(double i: averageTurretX){
      average += i;
    }
    average /= 6;
    SmartDashboard.putNumber("Average", average);

    if(joystick.getRawButton(1)){
      if(average<-.5) {
        robot.setTurretPowerPct(Math.sqrt(average)/12);
      }else if(average>.5) {
        robot.setTurretPowerPct(-Math.sqrt(average)/12);
      }else{
        robot.setTurretPowerPct(0.0);
      }
    }else{
      if(joystick.getRawButton(3)){
        robot.setTurretPowerPct(-0.1);
      }else if(joystick.getRawButton(4)){
        robot.setTurretPowerPct(0.1);
      }else{
        robot.setTurretPowerPct(0);
      }

    }

  }

  @Override public void disabledInit() {}

  @Override public void disabledPeriodic() {}

  @Override public void testInit() {}

  @Override public void testPeriodic() {}
}
