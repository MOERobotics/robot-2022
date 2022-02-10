// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generic.Falcon;
import frc.robot.generic.GenericAutonomous;
import frc.robot.generic.GenericRobot;
import frc.robot.generic.TurretBot;

public class Robot extends TimedRobot {

  GenericRobot robot = new Falcon();
  Joystick joystick = new Joystick(0);
  GenericAutonomous autonomous = new SimpleCTerminal(); //used to be autoArc()

  @Override public void robotInit() {}

  @Override public void robotPeriodic() {
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

    SmartDashboard.putNumber("Joystick raw X", joystick.getX());
    SmartDashboard.putNumber("Joystick raw Y", joystick.getY());

  }

  @Override public void autonomousInit() {
    autonomous.autonomousInit(robot);
  }

  @Override public void autonomousPeriodic() {
    autonomous.autonomousPeriodic(robot);
  }

  @Override public void teleopInit() { }

  @Override public void teleopPeriodic() {
    double x = joystick.getX();
    double y = -joystick.getY();

    robot.drivePercent(y+x,y-x);

    /*if(joystick.getRawButton(1)){
      robot.setShooterPowerPct(0.2, 0.2);
    }

    if(joystick.getRawButton(2)){
      robot.setCollectorIntakePercentage(0.2);
    }

    if(joystick.getRawButton(3)){
      robot.setTurretPowerPct(-0.15);
    }
    if(joystick.getRawButton(4)){
      robot.setTurretPowerPct(0.15);
    }*/

  }

  @Override public void disabledInit() {}

  @Override public void disabledPeriodic() {}

  @Override public void testInit() {}

  @Override public void testPeriodic() {}
}
