package frc.robot.generic;

public interface GenericRobot {


	public void drivePercent(
		double leftPercent,
		double rightPercent
	);

	public void driveRPM(
		double leftRPM,
		double rightRPM
	);


	public double getDriveLeftPercentage();
	public double getDriveRightPercentage();

	public double getDriveLeftRPM();
	public double getDriveRightRPM();

	public default double getDriveDistanceInchesLeft(){
		return encoderTicksLeftDrive()/encoderLeftDriveTicksPerInch();
	}

	public default double getDriveDistanceInchesRight(){
		return encoderTicksRightDrive()/encoderRightDriveTicksPerInch();
	}
	
	public default double encoderLeftDriveTicksPerInch(){
		System.out.println("I don't have an encoder");
		return 1.0;
	}

	public default double encoderRightDriveTicksPerInch(){
		System.out.println("I don't have an encoder");
		return 1.0;
	}
	
	public default double encoderTicksLeftDrive(){
		System.out.println("I don't have an encoder");
		return 0;
	}

	public default double encoderTicksRightDrive(){
		System.out.println("I don't have an encoder");
		return 0;
	}

	public default double getYee(){
		System.out.println("I don't have a navX");
		return 0;
	};

	public default double getPitch(){
		System.out.println("I don't have a navX");
		return 0;
	}

	public default double getRoll(){
		System.out.println("I don't have a navX");
		return 0;
	}

	public double getLinearVelocity();

	public enum TeamColor {
		RED,
		BLUE,
		UNKNOWN;
	}

	public default boolean getUpperCargo(){
		System.out.println("robot is colorblind");
		return false;
	}
	public default boolean getLowerCargo(){
		System.out.println("robot is colorblind");
		return false;
	}

	public default int getCargoCount(){
		System.out.println("robot can't count");
		return 0;
	}

	public void setCollectorIntakePercentage(
		double percentage
	);
	public double getCollectorIntakePercentage();

	public void setIndexerIntakePercentage(
			double percentage
	);
	public double getIndexerIntakePercentage();

	public default boolean hasFoundCargo(){
		System.out.println("robot has tunnel vision");
		return false;
	}

	public default TeamColor getFoundCargoColor(){
		System.out.println("Robot is colorblind");
		return TeamColor.UNKNOWN;
	}

	public default boolean isTargetFound() {
		return false;
	}

	public double getTargetX();
	public double getTargetY();
	public double getTargetDistance();
	public double getTargetAngle();

	public double getTurretAngle();
	public default double encoderTurretTicksPerDegree(){
		System.out.println("I don't have a turret encoder");
		return 1.0;
	}
	public default double getTurretAngleDegrees(){
		return getTurretAngle()/encoderTurretTicksPerDegree();
	}


	public void setTurretAngleRelative(double angleChange);
	public void setTurretAngleAbsolute();
	public void setTurretPowerPct(double powerPct);
	public double getTurretPowerPct();

	public double getTurretPitchAngle();
	public double getTurretPitchPowerPct();
	public void setTurretPitchAngle();
	public void setTurretPitchPowerPct();

	public double getShooterRPMTop();
	public double getShooterRPMBottom();
	public double getShooterPowerPctTop();
	public double getShooterPowerPctBottom();
	public double getShooterTargetDistance();
	public double getShooterTargetHeight();
	public void setShooterRPM(double topRPM, double bottomRPM);
	public void setShooterRPMTop(double rpm);
	public void setShooterRPMBottom(double rpm);
	public void setShooterPowerPct(double topPCT, double bottomPCT);
	public void setShooterPowerPctTop(double percentage);
	public void setShooterPowerPctBottom(double percentage);
	public void setShooterTargetDistance(double length, double height);

	public void raiseCollector();
	public void lowerCollector();

	public void turnOnPTO();
	public void turnOffPTO();

	public void setArmsForward();
	public void setArmsBackward();



}
