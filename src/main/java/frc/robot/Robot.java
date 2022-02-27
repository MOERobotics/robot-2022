// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.*;
import frc.robot.autonomous.GenericAutonomous;
import frc.robot.command.*;
import frc.robot.generic.GenericRobot;
import frc.robot.generic.Lightning;

import java.util.Arrays;
import java.util.Map;
import java.util.function.Function;
import java.util.stream.Collectors;

public class Robot extends TimedRobot {

  //Instantiate autonomous once, don't create unnecessary duplicates
  //Add new Autos here when they're authored
  public static final GenericAutonomous
          autoArc = new autoArc(),
          simpleATerminal = new BallAtoTerminal(),
          simpleBTerminal = new BallBtoTerminal(),
          simpleCTerminal = new BallCtoTerminal(),
          CTerminalReturn = new BallCtoTerminalReturn(),
          simpleB         = new BallSimpleB(),
          calibration = new Calibration();

  GenericRobot robot = new Lightning();
  Joystick joystick = new Joystick(0);
  GenericCommand command = new Hang();
  Joystick xbox = new Joystick(1);
  GenericAutonomous autonomous = calibration;


  int averageTurretXSize = 2;
  double[] averageX = new double[averageTurretXSize];


  int counter = 0;
  double average;
  double turretPower;
  boolean hang = false;
  int count = 0;
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
  double pitchChange = 0;

  double defCollectorPower = 1;
  double defIndexerPower = 1;
  double curCollector;
  double curIndexer;

  double targetRPM = 0;
  double turretPitch = 0;


  PIDController turretPIDController;


  @Override
  public void robotInit() {
    robot.setTurretPitchPosition(0);
  }

  @Override
  public void robotPeriodic() {

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

    SmartDashboard.putNumber("Turret direction angle ticks", robot.getTurretAngle());
    SmartDashboard.putNumber("Turret direction angle degrees", robot.getTurretAngleDegrees());
    SmartDashboard.putNumber("Alternate turret angle degrees", robot.getAlternateTurretAngle());

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

    SmartDashboard.putNumber("leftEncoderRaw", robot.encoderTicksLeftDriveA());
    SmartDashboard.putNumber("rightEncoderRaw", robot.encoderTicksRightDriveA());
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
    robot.setTurretPitchPosition(0);
    turretPIDController = new PIDController(robot.turretPIDgetP(), robot.turretPIDgetI(), robot.turretPIDgetD());
    hang = false;
    count = 0;
  }

  @Override
  public void teleopPeriodic() {
    turretPower = 0;


    switch (POVDirection.getDirection(xbox.getPOV())) {
      case NORTH: //MEDIUM SHOT RANGE
        targetRPM = 4000;
        turretPitch = 0.38;
        break;
      case EAST:
        targetRPM = 4000;
        turretPitch = 0.00;
        break;
      case SOUTH: ////CLOSE SHOT
        targetRPM = 3500;
        turretPitch = 0.12;
        break;
      case WEST:
        targetRPM = 5500;
        turretPitch = 0.00;
        break;
    }

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

      //////////////////////////////////////////////////DRIVETRAIN CONTROL

      joystickX = joystick.getX();
      joystickY = -joystick.getY();

      if (joystickY > -cutoff && joystickY < cutoff) {
        joystickY = 0;
      }
      if (joystickX > -cutoff && joystickX < cutoff) {
        joystickX = 0;
      }

      if (!robot.getPTOState()) {
        driveLeft = (joystickY + joystickX) * scaleFactor;
        driveRight = (joystickY - joystickX) * scaleFactor;
      }

      //////////////////////////////////////////////DRIVETRAIN CONTROL ENDS


      //////////////////////////////////////////////////////////TURRET CONTROL

      counter = (counter + 1) % averageTurretXSize;
      averageX[counter] = robot.getTargetX();

      average = 0;
      for (int i = 0; i < averageTurretXSize; i++)
        average += averageX[i];
      average /= averageTurretXSize;

      if ((xbox.getRawAxis(2) > 0.10) & robot.isTargetFound()) { ////////////AUTO-AIM
        turretPower = turretPIDController.calculate(average);
      } else {
        turretPIDController.reset();
        if (xbox.getRawButton(6)) {
          turretPower = -0.45;
        } else if (xbox.getRawButton(5)) {
          turretPower = 0.45;
        } else {
          turretPower = 0.0;
        }
      }
      /////////////////////////////////////////////////////TURRET CONTROL ENDS


      /////////////////////////////////////////////////////CLIMBER WHEN PTO

      if (robot.getPTOState()) {
        if (xbox.getRawAxis(leftAxis) > tolerance) {
          driveLeft = defaultClimbPower;
        } else if (xbox.getRawAxis(leftAxis) < -tolerance) {
          driveLeft = -defaultClimbPower;
        }

        if (xbox.getRawAxis(rightAxis) > tolerance) {
          driveRight = defaultClimbPower;
        } else if (xbox.getRawAxis(rightAxis) < -tolerance) {
          driveRight = -defaultClimbPower;
        }
      }
      /////////////////////////////////////////////////////CLIMBER CODE ENDS


      //////////////////////////////////////////////////////////SHOOTER CODE BEGINS

      if (xbox.getRawButton(3)) {
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

      double deadzone = 0.1;
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
    joystick.getRawButtonPressed(8);
    hang = false;
    count = 0;
    if (joystick.getRawButton(1)) {
      robot.resetAttitude();
      robot.resetEncoders();
    }

    if (joystick.getRawButton(4)) autonomous = autoArc;
    if (joystick.getRawButton(5)) autonomous = simpleATerminal;
    if (joystick.getRawButton(6)) autonomous = simpleBTerminal;
    if (joystick.getRawButton(7)) autonomous = simpleCTerminal;
    if (joystick.getRawButton(9)) autonomous = CTerminalReturn;
    if (joystick.getRawButton(11)) autonomous = simpleB;
    if (joystick.getRawButton(12)) autonomous = calibration;

  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {

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