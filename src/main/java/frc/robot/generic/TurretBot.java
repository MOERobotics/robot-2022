package frc.robot.generic;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.*;

import static com.revrobotics.CANSparkMax.IdleMode.kBrake;
import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;

public class TurretBot implements GenericRobot {
	public static final double TICKS_PER_INCH_DRIVE = 0.75;
	public static final double TICKS_PER_DEGREE_TURRET = 116;
	public static final double TICKS_PER_REVOLUTION_SHOOTERA = 116;
	public static final double TICKS_PER_REVOLUTION_SHOOTERB = 116;

	AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 50);

	CANSparkMax indexer   = new CANSparkMax(40, kBrushless);
	CANSparkMax speeeen   = new CANSparkMax(41, kBrushless);
	CANSparkMax shooterA  = new CANSparkMax(42, kBrushless);
	CANSparkMax shooterB  = new CANSparkMax(49, kBrushless);
	CANSparkMax collector = new CANSparkMax(43, kBrushless);

	CANSparkMax leftMotorA = new CANSparkMax(20, kBrushless);
	CANSparkMax leftMotorB = new CANSparkMax(1, kBrushless);
	CANSparkMax rightMotorA = new CANSparkMax(14, kBrushless);
	CANSparkMax rightMotorB = new CANSparkMax(15, kBrushless);

	RelativeEncoder encoderRight     = rightMotorA.getEncoder();
	RelativeEncoder encoderLeft      = leftMotorA.getEncoder();
	RelativeEncoder encoderTurret    = speeeen.getEncoder();
	RelativeEncoder encoderShootA    = shooterA.getEncoder();
	RelativeEncoder encoderShootB    = shooterB.getEncoder();

	SparkMaxPIDController shooterAPIDController = shooterA.getPIDController();
	SparkMaxPIDController shooterBPIDController = shooterB.getPIDController();

	Solenoid shifter = new Solenoid(PneumaticsModuleType.CTREPCM,0);
	Servo elevationLeft  = new Servo(0);
	Servo elevationRight = new Servo(1);

	DigitalInput homeSensor = new DigitalInput(6);

	public TurretBot(){
		boolean invertLeft = false;
		boolean invertRight = true;
		leftMotorA.setInverted(invertLeft);
		leftMotorB.setInverted(invertLeft);
		rightMotorA.setInverted(invertRight);
		rightMotorB.setInverted(invertRight);

		leftMotorA.setIdleMode(kBrake);
		leftMotorB.setIdleMode(kBrake);
		rightMotorA.setIdleMode(kBrake);
		rightMotorB.setIdleMode(kBrake);

	}

	@Override
	public double turretPIDgetP(){
		return 3.0e-2;
	}

	@Override
	public double turretPIDgetI(){
		return 0;
	}
	@Override
	public double turretPIDgetD(){
		return 3.0e-4;
	}

	@Override
	public void drivePercent(double leftPercent, double rightPercent) {
		leftMotorA.set(leftPercent);
		leftMotorB.set(leftPercent);
		rightMotorA.set(rightPercent);
		rightMotorB.set(rightPercent);
	}


	@Override
	public double getDriveLeftPercentage() {
		return leftMotorA.get();
	}

	@Override
	public double getDriveRightPercentage() {
		return rightMotorA.get();
	}

	@Override
	public double getDriveLeftRPM() {
		return encoderLeft.getVelocity()/encoderLeftDriveTicksPerInch();
	}

	@Override
	public double getDriveRightRPM() {
		return encoderRight.getVelocity()/encoderRightDriveTicksPerInch();
	}

	@Override
	public double getDriveDistanceInchesLeft() {
		return encoderTicksLeftDriveA() / encoderLeftDriveTicksPerInch();
	}

	@Override
	public double getDriveDistanceInchesRight() {
		return encoderTicksRightDriveA() / encoderRightDriveTicksPerInch();
	}

	@Override
	public double encoderLeftDriveTicksPerInch() {
		return TICKS_PER_INCH_DRIVE;
	}

	@Override
	public double encoderRightDriveTicksPerInch() {
		return TICKS_PER_INCH_DRIVE;
	}

	@Override
	public double encoderTicksLeftDriveA() {
		return encoderLeft.getPosition();
	}

	@Override
	public double encoderTicksRightDriveA() {
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
		//TODO
		return 0;
	}

	@Override
	public double getPIDmaneuverP() {
		return 1.0e-2;
	}

	@Override
	public double getPIDmaneuverI() {
		return 0;
	}

	@Override
	public double getPIDmaneuverD() {
		return 1.0e-4;
	}

	@Override
	public double getPIDpivotP() {
		return 1.5e-2;
	}

	@Override
	public double getPIDpivotI() {
		return 0;
	}

	@Override
	public double getPIDpivotD() {
		return 0;
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
	public boolean getUpperCargo() {
		return GenericRobot.super.getUpperCargo();
	}

	@Override
	public boolean getLowerCargo() {
		return GenericRobot.super.getLowerCargo();
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
	public void setIndexerIntakePercentage(double percentage) {
		indexer.set(percentage);
	}

	@Override
	public double getIndexerIntakePercentage() {
		return indexer.get();
	}

	@Override
	public boolean hasFoundCargo() {
		return GenericRobot.super.hasFoundCargo();
	}

	@Override
	public TeamColor getFoundCargoColor() {
		return GenericRobot.super.getFoundCargoColor();
	}

	@Override
	public boolean isTargetFound() {
		return GenericRobot.super.isTargetFound();
	}


	@Override
	public double getTurretAngle() {
		return encoderTurret.getPosition() / TICKS_PER_DEGREE_TURRET;
	}



	@Override
	public void setTurretPowerPct(double powerPct) {
		speeeen.set(powerPct);
	}

	@Override
	public double getTurretPowerPct() {
		return speeeen.get();
	}

	@Override
	public double getShooterRPMTop() {
		return encoderShootA.getVelocity() / TICKS_PER_REVOLUTION_SHOOTERA;
	}

	@Override
	public double getShooterRPMBottom() {
		return encoderShootB.getVelocity() / TICKS_PER_REVOLUTION_SHOOTERB;
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
		setShooterRPMTop(topRPM);
		setShooterRPMBottom(bottomRPM);
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
		setShooterPowerPctTop(topPCT);
		setShooterPowerPctBottom(bottomPCT);
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
		//TODO
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
