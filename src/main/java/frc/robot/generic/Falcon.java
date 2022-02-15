package frc.robot.generic;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.AnalogInput;

public class Falcon implements GenericRobot {

    AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 50);

    CANSparkMax leftDriveA = new CANSparkMax(13, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax leftDriveB = new CANSparkMax(14, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax leftDriveC = new CANSparkMax(15, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax rightDriveA = new CANSparkMax(20, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax rightDriveB = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax rightDriveC = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);

    CANSparkMax climberPort = new CANSparkMax(12, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax climberStarboard = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax generatorShift = null;//new CANSparkMax(11, CANSparkMaxLowLevel.MotorType.kBrushless);

    CANSparkMax shooterA = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
    SparkMaxPIDController shooterAPIDController = shooterA.getPIDController();
    CANSparkMax shooterB = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
    SparkMaxPIDController shooterBPIDController = shooterB.getPIDController();
    CANSparkMax indexer = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax escalator = new CANSparkMax(7, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax angleAdj = new CANSparkMax(8, CANSparkMaxLowLevel.MotorType.kBrushless);

    CANSparkMax controlPanel = new CANSparkMax(9, CANSparkMaxLowLevel.MotorType.kBrushless);

    CANSparkMax collector = new CANSparkMax(10, CANSparkMaxLowLevel.MotorType.kBrushless);

    RelativeEncoder encoderRight = rightDriveA.getEncoder();
    RelativeEncoder encoderLeft = leftDriveA.getEncoder();
    RelativeEncoder encoderShootA = shooterA.getEncoder();
    RelativeEncoder encoderShootB = shooterB.getEncoder();

    RelativeEncoder encoderClimbPort = climberPort.getEncoder();
    RelativeEncoder encoderClimbStarboard = climberStarboard.getEncoder();


    Solenoid starboardShooter = new Solenoid(PneumaticsModuleType.CTREPCM, 7);
    Solenoid portShooter = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    Solenoid starboardEscalator = new Solenoid(PneumaticsModuleType.CTREPCM, 6);
    Solenoid portEscalator = new Solenoid(PneumaticsModuleType.CTREPCM, 1);


    private SparkMaxLimitSwitch angleAdjusterDigitalInputForward;
    private SparkMaxLimitSwitch angleAdjusterDigitalInputReverse;
    private AnalogInput input = new AnalogInput(0);
    private AnalogPotentiometer elevation = new AnalogPotentiometer(input, 180, 90);

    DigitalInput escalatorSensorLow = new DigitalInput(1);
    DigitalInput escalatorSensorMedium = new DigitalInput(6);
    DigitalInput escalatorSensorMediumHigh = new DigitalInput(4);
    DigitalInput escalatorSensorHigh = new DigitalInput(3);


    double aspectRatioTargDist;

    public Falcon() {

        rightDriveA.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightDriveB.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightDriveC.setIdleMode(CANSparkMax.IdleMode.kBrake);

        leftDriveA .setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftDriveB .setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftDriveC .setIdleMode(CANSparkMax.IdleMode.kBrake);

        rightDriveA.setInverted(true);
        rightDriveB.setInverted(true);
        rightDriveC.setInverted(true);

        leftDriveA.setInverted(false);
        leftDriveB.setInverted(false);
        leftDriveC.setInverted(false);

        collector.setInverted(true);

        escalator.setIdleMode(CANSparkMax.IdleMode.kBrake);

        shooterA.setIdleMode(CANSparkMax.IdleMode.kCoast);
        shooterB.setIdleMode(CANSparkMax.IdleMode.kCoast);

    }

    @Override
    public void drivePercent(double leftPercent, double rightPercent) {
        leftDriveA.set(leftPercent);
        leftDriveB.set(leftPercent);
        leftDriveC.set(leftPercent);

        rightDriveA.set(rightPercent);
        rightDriveB.set(rightPercent);
        rightDriveC.set(rightPercent);

    }



    @Override
    public double getDriveLeftPercentage() {
        return leftDriveA.get();
    }

    @Override
    public double getDriveRightPercentage() {
        return rightDriveA.get();
    }

    @Override
    public double getDriveLeftRPM() {
        return leftDriveA.getEncoder().getVelocity();
    }

    @Override
    public double getDriveRightRPM() {
        return rightDriveA.getEncoder().getVelocity();
    }

    @Override
    public double getDriveDistanceInchesLeft() {
        return encoderTicksLeftDrive()/encoderLeftDriveTicksPerInch();
    }

    @Override
    public double getDriveDistanceInchesRight() {
        return encoderTicksRightDrive()/encoderRightDriveTicksPerInch();
    }

    @Override
    public double encoderLeftDriveTicksPerInch() {
        return .306;
    }

    @Override
    public double encoderRightDriveTicksPerInch() {
        return .306;
    }

    @Override
    public double encoderTicksLeftDrive() {
        return encoderLeft.getPosition();
    }

    @Override
    public double encoderTicksRightDrive() {
        return encoderRight.getPosition();
    }

    @Override
    public double getYaw() {
        return navx.getYaw();
    }

    @Override
    public double getPitch() {
        return navx.getPitch();
    }

    @Override
    public double getRoll() {
        return navx.getRoll();
    }

    @Override
    public double getLinearVelocity() {
        return 0;
    }

    @Override
    public double getPIDmaneuverP() {
        return 5.5e-3;
    }

    @Override
    public double getPIDmaneuverI() {
        return 1.0e-10;
    }

    @Override
    public double getPIDmaneuverD() {
        return 1.0e-4;
    }

    @Override
    public double getPIDpivotP() {
        return 4.0e-2;
    }

    @Override
    public double getPIDpivotI() {
        return 1.0e-2;
    }

    @Override
    public double getPIDpivotD() {
        return 1.0e-4;
    }

    @Override
    public void resetEncoders() {
        encoderLeft.setPosition(0.0);
        encoderRight.setPosition(0.0);
    }

    @Override
    public void resetAttitude() {
        navx.reset();
    }


    @Override
    public void setCollectorIntakePercentage(double percentage) {
        collector.set(percentage);
    }

    @Override
    public double getCollectorIntakePercentage() {
        return collector.get();
    }

    @Override
    public double getTargetDistance() {
        return getTargetY() * aspectRatioTargDist;
    }



    @Override
    public double getShooterRPMTop() {
        return shooterA.getEncoder().getVelocity();
    }

    @Override
    public double getShooterRPMBottom() {
        return shooterB.getEncoder().getVelocity();
    }

    @Override
    public double getShooterPowerPctTop() {
        return shooterA.get();
    }

    @Override
    public double getShooterPowerPctBottom() {
        return shooterB.get();
    }


    @Override
    public void setShooterRPM(double topRPM, double bottomRPM) {
        shooterAPIDController.setReference(topRPM, CANSparkMax.ControlType.kVelocity);
        shooterBPIDController.setReference(bottomRPM, CANSparkMax.ControlType.kVelocity);
    }

    @Override
    public void setShooterRPMTop(double rpm) {
        shooterAPIDController.setReference(rpm, CANSparkMax.ControlType.kVelocity);
    }

    @Override
    public void setShooterRPMBottom(double rpm) {
        shooterBPIDController.setReference(rpm, CANSparkMax.ControlType.kVelocity);
    }

    @Override
    public void setShooterPowerPct(double topPCT, double bottomPCT) {
        shooterA.set(topPCT);
        shooterB.set(bottomPCT);
    }

    @Override
    public void setShooterPowerPctTop(double percentage) {
        shooterA.set(percentage);
    }

    @Override
    public void setShooterPowerPctBottom(double percentage) {
        shooterB.set(percentage);
    }



}



