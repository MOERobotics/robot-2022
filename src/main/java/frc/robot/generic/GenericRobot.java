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

	public default double getYaw(){
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

	double getPIDmaneuverP();

	double getPIDmaneuverI();

	double getPIDmaneuverD();

	double getPIDpivotP();

	double getPIDpivotI();

	double getPIDpivotD();

	void resetEncoders();

	void resetAttitude();

	public enum TeamColor {
		RED,
		BLUE,
		UNKNOWN;
	}

	public default TeamColor getCargoColor(){
		System.out.println("robot is colorblind");
		return TeamColor.UNKNOWN;
	}

	public default int getCargoCount(){
		System.out.println("robot can't count");
		return 0;
	}

	public void setCollectorIntakePercentage(
		double percentage
	);
	public double getCollectorIntakePercentage();

	public default boolean hasFoundCargo(){
		System.out.println("robot has tunnel vision");
		return false;
	}

	public default TeamColor getFoundCargoColor(){
		System.out.println("Robot is colorblind");
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
	public double getTargetDistance();
	public double getTargetAngle();

	public double getTurretAngle();
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

	public default double turretPIDgetP(){
		return 0;
	}
	public  default double turretPIDgetI(){
		return 0;
	}
	public  default double turretPIDgetD(){
		return 0;
	}






}
