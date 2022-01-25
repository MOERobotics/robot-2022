package frc.robot.generic;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.*;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;

public class TurretBot implements GenericRobot {
	public static final double TICKS_PER_INCH_DRIVE = 116;
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

	CANEncoder encoderRight     = rightMotorA.getEncoder();
	CANEncoder encoderLeft      = leftMotorA.getEncoder();
	CANEncoder encoderTurret    = speeeen.getEncoder();
	CANEncoder encoderShootA    = shooterA.getEncoder();
	CANEncoder encoderShootB    = shooterB.getEncoder();

	CANPIDController shooterAPIDController = shooterA.getPIDController();
	CANPIDController shooterBPIDController = shooterB.getPIDController();

	Solenoid shifter = new Solenoid(PneumaticsModuleType.CTREPCM,0);
	Servo elevationLeft = new Servo(0);
	Servo       elevationRight = new Servo(1);

	DigitalInput homeSensor = new DigitalInput(6);

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
		return encoderTicksLeftDrive() / encoderLeftDriveTicksPerInch();
	}

	@Override
	public double getDriveDistanceInchesRight() {
		return encoderTicksRightDrive() / encoderRightDriveTicksPerInch();
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
	public double encoderTicksLeftDrive() {
		return encoderLeft.getPosition();
	}

	@Override
	public double encoderTicksRightDrive() {
		return encoderRight.getPosition();
	}

	@Override
	public double getYee() {
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
	public TeamColor getCargoColor() {
		return GenericRobot.super.getCargoColor();
	}

	@Override
	public int getCargoCount() {
		return GenericRobot.super.getCargoCount();
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
	public int getTargetX() {
		//TODO
		return 0;
	}

	@Override
	public int getTargetY() {
		//TODO
		return 0;
	}

	@Override
	public int getTargetDistance() {
		//TODO
		return 0;
	}

	@Override
	public double getTargetAngle() {
		//TODO
		return 0;
	}

	@Override
	public double getTurretAngle() {
		return encoderTurret.getPosition() / TICKS_PER_DEGREE_TURRET;
	}

	@Override
	public void setTurretAngleRelative() {
		//TODO

	}

	@Override
	public void setTurretAngleAbsolute() {
		//TODO

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
	public double getTurretPitchAngle() {
		//TODO
		return 0;
	}

	@Override
	public double getTurretPitchPowerPct() {
		//TODO
		return 0;
	}

	@Override
	public void setTurretPitchAngle() {
		//TODO

	}

	@Override
	public void setTurretPitchPowerPct() {
		//TODO

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
	public double getShooterTargetDistance() {
		//TODO
		return 0;
	}

	@Override
	public double getShooterTargetHeight() {
		//TODO
		return 0;
	}

	@Override
	public void setShooterRPM(double topRPM, double bottomRPM) {
		setShooterRPMTop(topRPM);
		setShooterRPMBottom(bottomRPM);
	}

	@Override
	public void setShooterRPMTop(double rpm) {
		shooterAPIDController.setReference(rpm, ControlType.kVelocity);
	}

	@Override
	public void setShooterRPMBottom(double rpm) {
		shooterBPIDController.setReference(rpm, ControlType.kVelocity);
	}

	@Override
	public void setShooterPowerPct(double top, double bottom) {
		setShooterPowerPctTop(top);
		setShooterPowerPctBottom(bottom);
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
}
