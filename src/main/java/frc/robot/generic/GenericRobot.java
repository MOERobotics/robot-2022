package frc.robot.generic;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public interface GenericRobot {

	NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
	NetworkTableEntry tx = table.getEntry("tx");
	NetworkTableEntry ty = table.getEntry("ty");
	NetworkTableEntry ta = table.getEntry("ta");
	NetworkTableEntry tv = table.getEntry("tv");


	public void drivePercent(
		double leftPercent,
		double rightPercent
	);

	public default void driveRPM(
		double leftRPM,
		double rightRPM
	) {
		//System.out.println("I don't have an encoder to measure my RPM");
	};


	public default double getDriveLeftPercentage(){
		return 0;
	}
	public default double getDriveRightPercentage(){
		return 0;
	}

	public default double getDriveLeftRPM(){
		return 0;
	};
	public default double getDriveRightRPM(){
		return 0;
	};

	public default double getDriveDistanceInchesLeft(){
		return encoderTicksLeftDrive()/encoderLeftDriveTicksPerInch();
	}

	public default double getDriveDistanceInchesRight(){
		return encoderTicksRightDrive()/encoderRightDriveTicksPerInch();
	}

	public default double encoderLeftDriveTicksPerInch(){
		//System.out.println("I don't have an encoder");
		return 1.0;
	}

	public default double encoderRightDriveTicksPerInch(){
		//System.out.println("I don't have an encoder");
		return 1.0;
	}

	public default double encoderTicksLeftDrive(){
		//System.out.println("I don't have an encoder");
		return 0;
	}

	public default double encoderTicksRightDrive(){
		//System.out.println("I don't have an encoder");
		return 0;
	}

	public default double getYaw(){
		//System.out.println("I don't have a navX");
		return 0;
	}

	public default double getPitch(){
		//System.out.println("I don't have a navX");
		return 0;
	}

	public default double getRoll(){
		//System.out.println("I don't have a navX");
		return 0;
	}

	public default double getLinearVelocity() {
		//System.out.println("I don't have a mouse");
		return 0;
	}

	public default double getPIDmaneuverP() {return 0; }

	public default double getPIDmaneuverI() {return 0; }

	public default double getPIDmaneuverD() {return 0; }

	public default double getPIDpivotP() {return 0; }

	public default double getPIDpivotI() {return 0; }

	public default double getPIDpivotD() {return 0; }

	public default void resetEncoders() {
		System.out.println("I don't have encoders");
	}

	public default void resetAttitude() {
		System.out.println("I don't have a navx");
	}

	public enum TeamColor {
		RED,
		BLUE,
		UNKNOWN;
	}

	public default boolean getUpperCargo(){
		//System.out.println("robot is colorblind");
		return false;
	}
	public default boolean getLowerCargo(){
		//System.out.println("robot is colorblind");
		return false;
	}
	public default void setCollectorIntakePercentage(
		double percentage
	) {
		//System.out.println("robot cannot collect");
	}
	public default double getCollectorIntakePercentage() {
		return 0;
	};

	public void setIndexerIntakePercentage(
			double percentage
	);
	public double getIndexerIntakePercentage();

	public default boolean hasFoundCargo(){
		//System.out.println("robot has tunnel vision");
		return false;
	}
	public default TeamColor getFoundCargoColor(){
		//System.out.println("Robot is colorblind");
		return TeamColor.UNKNOWN;
	}

	public default boolean isTargetFound() {
		if (tv.getDouble(0.0) != 0){
			return true;
		}
		else{
			return false;
		}
	}

	public default double getTargetX(){
		return tx.getDouble(0.0);
	}
	public default double getTargetY(){
		return ty.getDouble(0.0);
	}

	public default double getTargetArea(){
		return ta.getDouble(0.0);
	}


	public default double getTargetDistance(){
		return 0;
	}
	public default  double getTargetAngle() {
		return 0;
	}


	//turret stuff

	public default double turretPIDgetP(){
		return 0;
	}
	public default double turretPIDgetI(){
		return 0;
	}
	public default double turretPIDgetD(){
		return 0;
	}

	public default double getTurretAngle(){
		return 0;
	}
	public default double encoderTurretTicksPerDegree(){
		//System.out.println("I don't have a turret encoder");
		return 1.0;
	}
	public default double getTurretAngleDegrees(){
		return getTurretAngle()/encoderTurretTicksPerDegree();
	}

	public default double getAlternateTurretAngle(){
		return 0;
	}
	public default double getAlternateTurretAngleDegrees(){
		return getAlternateTurretAngle()/encoderTurretTicksPerDegree();
	}


	public default void setTurretAngleRelative(double angleChange){
		//System.out.println("I don't have a turret");
	}
	public default void setTurretAngleAbsolute(){
		//System.out.println("I don't have a turret");
	}

	public default void setTurretPowerPct(double powerPct){
		//System.out.println("I don't have a turret");
	}
	public default double getTurretPowerPct(){
		return 0;
	}

	public default double getTurretPitchAngle(){
		return 0;
	}
	public default double getTurretPitchPowerPct(){
		return 0;
	}

	public default void setTurretPitchAngle(){
		//System.out.println("I don't have a turret");
	}
	public default void setTurretPitchPowerPct(double speed){
		//System.out.println("I don't have a collector");
	}

	public default double getShooterRPMTop(){
		return 0;
	}
	public default double getShooterRPMBottom(){
		return 0;
	}
	public default double getShooterPowerPctTop(){
		return 0;
	}
	public default double getShooterPowerPctBottom(){
		return 0;
	}
	public default double getShooterTargetDistance(){
		return 0;
	}
	public default double getShooterTargetHeight(){
		return 0;
	}
	public default double getShooterTargetRPM() { return 0; }

	public default void setShooterRPM(double topRPM, double bottomRPM){
		//System.out.println("I don't have a shooter");
	}
	public default void setShooterRPMTop(double rpm){
		//System.out.println("I don't have a shooter");
	}
	public default void setShooterRPMBottom(double rpm) {
		//System.out.println("I don't have a shooter");
	}
	public default void setShooterPowerPct(double topPCT, double bottomPCT) {
		//System.out.println("I don't have a shooter");
	}
	public default void setShooterPowerPctTop(double percentage) {
		//System.out.println("I don't have a shooter");
	}
	public default void setShooterPowerPctBottom(double percentage) {
		//System.out.println("I don't have a shooter");
	}
	public default void setShooterTargetDistance(double length, double height) {
		//System.out.println("I don't have a shooter");
	}

	public default boolean getFloorSensorLeft(){
		return false;
	}
	public default boolean getFloorSensorRight(){
		return false;
	}


	public default void raiseCollector() {
		//System.out.println("I don't have a collector");
	}

	public default void lowerCollector() {
		//System.out.println("I don't have a collector");
	}

	public default void turnOnPTO() {
		//System.out.println("I don't have a PTO");
	}
	public default void turnOffPTO() {
		//System.out.println("I don't have a PTO");
	}

	public default void setArmsForward() {
		//System.out.println("I don't have a climber");
	}
	public default void setArmsBackward() {
		//System.out.println("I don't have a climber");
	}

	//returns true if PTO set to arms, return false if PTO set to drive
	public default boolean getPTOState(){
		return false;
	}

	public default boolean getClimbSensorLeft(){
		return false;
	}
	public default boolean getClimbSensorRight(){
		return false;
	}


	public default long getShootReadyTimer(){
		//System.out.println("Robot doesn't check if it's ready to shoot");
		return System.currentTimeMillis();
	}
	public default void shooterNotReady(){
		//System.out.println("Robot doesn't check if it's ready to shoot");
	}
	public default boolean isReadyToShoot(){
		return isReadyToShoot(0.02, 100);
	}
	public default boolean isReadyToShoot(double tolerance, double time){
		double shooterRPM = getShooterRPMBottom();
		double targetRPM = getShooterTargetRPM();

		//absolute percent error between actual shooter and target
		double error = Math.abs( (shooterRPM - targetRPM) / targetRPM);

		if(error > tolerance) shooterNotReady();

		//we haven't called shooterNotReady() in the last "time" milliseconds
		if(System.currentTimeMillis() - getShootReadyTimer() > time) return true;
		return false;
	}

	public default boolean isActivelyShooting(){
		return false;
	}
	public default void setActivelyShooting(boolean isShooting){

	}
	public default void raiseClimberArms(){
		//System.out.println("I don't have a climber");
	}
	public default void lowerClimberArms(){
		//System.out.println("I don't have a climber");
	}
	public default void armPower(double power){
		//System.out.println("Elbow Grease");
	}
	public default double armHeight(){
		return 0.0;
		//System.out.println("I don't have a climber");
	}

	public default boolean armInContact(){
		return false;
		//System.out.println("I don't have a climber");
	}

	public default boolean inTheRightPlace(){
		return false;
		//System.out.println("I don't have a climber");
	}

}
