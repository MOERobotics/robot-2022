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
import frc.robot.generic.TurretBot;

public class Robot extends TimedRobot {

  GenericRobot robot = new TurretBot();
  Joystick joystick = new Joystick(0);


  int averageTurretXSize = 2;
  double[] averageTurretX = new double [averageTurretXSize];

  double turretx;
  double turrety;
  double turretarea;
  double turretv;
  int counter = 0;


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

    SmartDashboard.putNumber("Left encoder", robot.getDriveDistanceInchesLeft());
    SmartDashboard.putNumber("Right encoder", robot.getDriveDistanceInchesRight());

    SmartDashboard.putNumber("Yee", robot.getYee());
    SmartDashboard.putNumber("Pitch", robot.getPitch());
    SmartDashboard.putNumber("Rollll", robot.getRoll());
    SmartDashboard.putNumber("Linear speed", robot.getLinearVelocity());

    SmartDashboard.putString("Held cargo", robot.getCargoColor().name());
    SmartDashboard.putNumber("Cargo inventory", robot.getCargoCount());
    SmartDashboard.putBoolean("Has detected cargo?", robot.hasFoundCargo());

    SmartDashboard.getNumber("Collector intake power", robot.getCollectorIntakePercentage());
    SmartDashboard.getBoolean("Sees target?", robot.isTargetFound());

    SmartDashboard.getNumber("Vision target x", robot.getTargetX());
    SmartDashboard.getNumber("Vision target y", robot.getTargetY());
    SmartDashboard.getNumber("Vision target angle", robot.getTargetAngle());
    SmartDashboard.getNumber("Vision target dist", robot.getTargetDistance());

    SmartDashboard.getNumber("Turret direction angle", robot.getTurretAngle());
    SmartDashboard.getNumber("Turret direction motor pct", robot.getTurretPowerPct());

    SmartDashboard.getNumber("Turret pitch angle", robot.getTurretPitchAngle());
    SmartDashboard.getNumber("Turret pitch motor pct", robot.getTurretPitchPowerPct());

    SmartDashboard.getNumber("Shooter top motor pct", robot.getShooterPowerPctTop());
    SmartDashboard.getNumber("Shooter bottom motor pct", robot.getShooterPowerPctBottom());

    SmartDashboard.getNumber("Shooter top motor rpm", robot.getShooterRPMTop());
    SmartDashboard.getNumber("Shooter bottom motor rpm", robot.getShooterRPMBottom());

    SmartDashboard.getNumber("Shooter calculate distance", robot.getShooterTargetDistance());
    SmartDashboard.getNumber("Shooter calculate height", robot.getShooterTargetHeight());



  }

  @Override public void autonomousInit() {}

  @Override public void autonomousPeriodic() {}

  @Override public void teleopInit() {


  }

  @Override public void teleopPeriodic() {
    double x = joystick.getX();
    double y = joystick.getY();

    robot.drivePercent(y+x,y-x);

   /* if(joystick.getRawButton(1)){
      robot.setShooterPowerPct(0.2, 0.2);
    }*/

    if(joystick.getRawButton(2)){
      robot.setCollectorIntakePercentage(0.2);
    }


    //Start of Daniel+Saiarun Turret test
    double average = 0;

    if(turretv !=0 ) {
      averageTurretX[counter % averageTurretXSize] = turretx;
      counter++;
    }
    average = 0;
    for(double i: averageTurretX){
      average += i;
    }
    average /= averageTurretXSize;
    SmartDashboard.putNumber("Average", average);

    double currentTurretPower = 0;

    if(joystick.getRawButton(1) && turretv !=0){
      double currentTurretPowerValue = -(Math.signum(average)*average*average)/30;
      if(currentTurretPowerValue >.2){
        currentTurretPowerValue = .2;
      } else if(currentTurretPowerValue <-.2){
        currentTurretPowerValue = -.2;
      }

      if(average< -1) {
        currentTurretPower = currentTurretPowerValue;
      }else if(average> 1) {
        currentTurretPower = currentTurretPowerValue;
      }else{
        currentTurretPower = 0;
      }
    }else{
      if(joystick.getRawButton(3)){
        currentTurretPower = -0.1;
      }else if(joystick.getRawButton(4)){
        currentTurretPower = 0.1;
      }else{
        currentTurretPower = 0;
      }
    }

    SmartDashboard.putNumber("currentTurretPower", currentTurretPower);
    robot.setTurretPowerPct(currentTurretPower);

  }

  @Override public void disabledInit() {}

  @Override public void disabledPeriodic() {}

  @Override public void testInit() {}

  @Override public void testPeriodic() {}
}
