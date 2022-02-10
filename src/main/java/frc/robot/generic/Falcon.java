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

	CANSparkMax leftDriveA      = new CANSparkMax(13, CANSparkMaxLowLevel.MotorType.kBrushless);
	CANSparkMax leftDriveB      = new CANSparkMax(14, CANSparkMaxLowLevel.MotorType.kBrushless);
	CANSparkMax leftDriveC      = new CANSparkMax(15, CANSparkMaxLowLevel.MotorType.kBrushless);
	CANSparkMax rightDriveA     = new CANSparkMax(20, CANSparkMaxLowLevel.MotorType.kBrushless);
	CANSparkMax rightDriveB     = new CANSparkMax( 1, CANSparkMaxLowLevel.MotorType.kBrushless);
	CANSparkMax rightDriveC     = new CANSparkMax( 2, CANSparkMaxLowLevel.MotorType.kBrushless);

	CANSparkMax climberPort = new CANSparkMax(12, CANSparkMaxLowLevel.MotorType.kBrushless);
	CANSparkMax climberStarboard = new CANSparkMax( 3, CANSparkMaxLowLevel.MotorType.kBrushless);
	CANSparkMax generatorShift  = null;//new CANSparkMax(11, CANSparkMaxLowLevel.MotorType.kBrushless);

	CANSparkMax shooterA                   = new CANSparkMax( 5, CANSparkMaxLowLevel.MotorType.kBrushless);
	SparkMaxPIDController shooterAPIDController = shooterA.getPIDController();
	CANSparkMax shooterB                   = new CANSparkMax( 4, CANSparkMaxLowLevel.MotorType.kBrushless);
	SparkMaxPIDController shooterBPIDController = shooterB.getPIDController();
	CANSparkMax indexer         = new CANSparkMax( 6, CANSparkMaxLowLevel.MotorType.kBrushless);
	CANSparkMax escalator       = new CANSparkMax( 7, CANSparkMaxLowLevel.MotorType.kBrushless);
	CANSparkMax angleAdj        = new CANSparkMax( 8, CANSparkMaxLowLevel.MotorType.kBrushless);

	CANSparkMax controlPanel    = new CANSparkMax( 9, CANSparkMaxLowLevel.MotorType.kBrushless);

	CANSparkMax collector       = new CANSparkMax(10, CANSparkMaxLowLevel.MotorType.kBrushless);

	RelativeEncoder encoderRight     = rightDriveA.getEncoder();
	RelativeEncoder encoderLeft      = leftDriveA.getEncoder();
	RelativeEncoder encoderShootA    = shooterA.getEncoder();
	RelativeEncoder encoderShootB    = shooterB.getEncoder();

	RelativeEncoder encoderClimbPort = climberPort.getEncoder();
	RelativeEncoder encoderClimbStarboard = climberStarboard.getEncoder();


	Solenoid starboardShooter = new Solenoid(PneumaticsModuleType.CTREPCM,7);
	Solenoid portShooter = new Solenoid(PneumaticsModuleType.CTREPCM,0);
	Solenoid starboardEscalator = new Solenoid(PneumaticsModuleType.CTREPCM,6);
	Solenoid portEscalator = new Solenoid(PneumaticsModuleType.CTREPCM,1);


	private SparkMaxLimitSwitch angleAdjusterDigitalInputForward;
	private SparkMaxLimitSwitch angleAdjusterDigitalInputReverse;
	private AnalogInput input = new AnalogInput(0);
	private AnalogPotentiometer elevation = new AnalogPotentiometer(input, 180, 90);

	DigitalInput escalatorSensorLow = new DigitalInput(1);
	DigitalInput escalatorSensorMedium = new DigitalInput(6);
	DigitalInput escalatorSensorMediumHigh = new DigitalInput(4);
	DigitalInput escalatorSensorHigh = new DigitalInput(3);


	public NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
	NetworkTableEntry tx = table.getEntry("tx");
	NetworkTableEntry ty = table.getEntry("ty");
	NetworkTableEntry ta = table.getEntry("ta");

	double aspectRatioTargDist;


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
	public void driveRPM(double leftRPM, double rightRPM) {

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
	public double getLinearVelocity() {
		return 0;
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
	public double getTargetX() {
		return tx.getDouble(0.0);
	}

	@Override
	public double getTargetY() {
		return ty.getDouble(0.0);
	}

	@Override
	public double getTargetDistance() {
		return getTargetY() * aspectRatioTargDist;
	}

	@Override
	public double getTargetAngle() {
		// do some math
		return 0;
	}

	@Override
	public double getTurretAngle() {
		return 0;
	}

	@Override
	public void setTurretAngleRelative(double angleChange) {

	}

	@Override
	public void setTurretAngleAbsolute() {

	}

	@Override
	public void setTurretPowerPct(double powerPct) {

	}

	@Override
	public double getTurretPowerPct() {
		return 0;
	}

	@Override
	public double getTurretPitchAngle() {
		return 0;
	}

	@Override
	public double getTurretPitchPowerPct() {
		return 0;
	}

	@Override
	public void setTurretPitchAngle() {

	}

	@Override
	public void setTurretPitchPowerPct() {

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
	public double getShooterTargetDistance() {
		return 0;
	}

	@Override
	public double getShooterTargetHeight() {
		return getTargetDistance(); //+ some fancy math stuff that I don't know right now
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

	@Override
	public void setShooterTargetDistance(double length, double height) {

	}

	@Override
	public void raiseCollector() { return; }

	@Override
	public void lowerCollector() { return; }

	@Override
	public void turnOnPTO() { return; }

	@Override
	public void turnOffPTO() { return; }

	@Override
	public void setArmsForward() { return; }

	@Override
	public void setArmsBackward() { return; }
}

