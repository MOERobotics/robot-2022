package frc.robot.generic;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;

public class Camoelot implements GenericRobot {
	double TICKS_PER_INCH_LEFT = 116.0;
	double TICKS_PER_INCH_RIGHT = 116.0;

	AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 50);

	private TalonSRX  leftMotorA = new TalonSRX(12);
	private TalonSRX  leftMotorB = new TalonSRX(13);
	private TalonSRX  leftMotorC = new TalonSRX(14);
	private TalonSRX rightMotorA = new TalonSRX( 1);
	private TalonSRX rightMotorB = new TalonSRX( 2);
	private TalonSRX rightMotorC = new TalonSRX( 3);

	private TalonSRX shooterA = new TalonSRX(15);
	private TalonSRX shooterB = new TalonSRX(10);

	private Encoder leftEncoder = new Encoder(0,1);
	private Encoder rightEncoder = new Encoder(2,3);




	int[] leftEncoderHistory = new int[5];
	int[] rightEncoderHistory = new int[5];


	public Camoelot(){
		boolean invertLeft = true;
		leftMotorA.setInverted(invertLeft);
		leftMotorB.setInverted(invertLeft);
		leftMotorC.setInverted(invertLeft);

		leftMotorA.setNeutralMode(NeutralMode.Brake);
		leftMotorB.setNeutralMode(NeutralMode.Brake);
		leftMotorC.setNeutralMode(NeutralMode.Brake);

		rightMotorA.setNeutralMode(NeutralMode.Brake);
		rightMotorB.setNeutralMode(NeutralMode.Brake);
		rightMotorC.setNeutralMode(NeutralMode.Brake);
	}

	@Override
	public void drivePercent(double leftPercent, double rightPercent) {
		leftMotorA.set(ControlMode.PercentOutput, leftPercent);
		leftMotorB.set(ControlMode.PercentOutput, leftPercent);
		leftMotorC.set(ControlMode.PercentOutput, leftPercent);

		rightMotorA.set(ControlMode.PercentOutput, rightPercent);
		rightMotorB.set(ControlMode.PercentOutput, rightPercent);
		rightMotorC.set(ControlMode.PercentOutput, rightPercent);
	}


	@Override
	public void driveRPM(double leftRPM, double rightRPM) {
	}


	@Override
	public double getDriveLeftPercentage() {
		return leftMotorA.getMotorOutputPercent();
	}

	@Override
	public double getDriveRightPercentage() {
		return rightMotorA.getMotorOutputPercent();
	}

	@Override
	public double getDriveLeftRPM() {
		return 0;
	}

	@Override
	public double getDriveRightRPM() {
		return 0;
	}

	//in inches per second
	@Override
	public double getLinearVelocity(){

		double leftInchesPerSec = leftEncoder.getRate() / encoderLeftDriveTicksPerInch();
		double rightInchesPerSec = rightEncoder.getRate() / encoderRightDriveTicksPerInch();

		//find speed of center of robot, which is average of left and right sides
		return (leftInchesPerSec + rightInchesPerSec) / 2;

	}

	@Override
	public double getPIDmaneuverP() {
		return 0;
	}

	@Override
	public double getPIDmaneuverI() {
		return 0;
	}

	@Override
	public double getPIDmaneuverD() {
		return 0;
	}


	@Override
	public void setIndexerIntakePercentage(double percentage) {
		System.out.println("Camelot cannot index");
	}

	@Override
	public double getIndexerIntakePercentage() {
		return 0;
	}

	@Override
	public double getPIDpivotP() {
		return 0;
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
		leftEncoder.reset();
		rightEncoder.reset();
	}

	@Override
	public void resetAttitude() {
		navx.reset();
	}

	@Override
	public double getYaw(){
		return navx.getYaw();
	}
	@Override
	public double getPitch(){
		return navx.getPitch();
	}
	@Override
	public double getRoll(){
		return navx.getRoll();
	}

	@Override
	public double encoderLeftDriveTicksPerInch(){
		return TICKS_PER_INCH_LEFT;
	}

	@Override
	public double encoderRightDriveTicksPerInch(){
		return TICKS_PER_INCH_RIGHT;
	}

	@Override
	public double encoderTicksLeftDrive(){
		return leftEncoder.get();
	}

	@Override
	public double encoderTicksRightDrive(){
		return rightEncoder.get();
	}

	@Override
	public void setCollectorIntakePercentage(double percentage) {

		System.out.println("Camelot cannot collect");
	}

	@Override
	public double getCollectorIntakePercentage() {
		return 0;
	}





	@Override
	public double getShooterPowerPctTop() {

		return shooterA.getMotorOutputPercent();
	}

	@Override
	public double getShooterPowerPctBottom() {
		return shooterB.getMotorOutputPercent();
	}

	@Override
	public double getShooterTargetDistance() {
		return 0;
	}

	@Override
	public double getShooterTargetHeight() {
		return 0;
	}

	@Override
	public void setShooterRPM(double topRPM, double bottomRPM) {
		setShooterRPMTop(topRPM);
		setShooterRPMBottom(bottomRPM);
	}

	@Override
	public void setShooterPowerPct(double topPCT, double bottomPCT) {
		setShooterPowerPctTop(topPCT);
		setShooterPowerPctBottom(bottomPCT);
	}

	@Override
	public void setShooterPowerPctTop(double percentage) {

		shooterA.set(ControlMode.PercentOutput, percentage);
	}

	@Override
	public void setShooterPowerPctBottom(double percentage) {

		shooterB.set(ControlMode.PercentOutput, percentage);
	}


}
