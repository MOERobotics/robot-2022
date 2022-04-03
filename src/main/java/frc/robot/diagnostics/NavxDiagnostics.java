package frc.robot.diagnostics;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.networktables.NetworkTable;
import lombok.Data;
import lombok.Value;

@Value
public class NavxDiagnostics implements DiagnosticsLoader {
	AHRS navx;
	@Override
	public boolean loadDiagnostics(NetworkTable targetTable) {
		if (!navx.isConnected()) return false;
		targetTable.getEntry("Version").setString(navx.getFirmwareVersion());
		targetTable.getEntry("Update Rate").setNumber(navx.getActualUpdateRate());
		targetTable.getEntry("Baro Pressure").setNumber(navx.getBarometricPressure());
		targetTable.getEntry("Pressure").setNumber(navx.getPressure());
		targetTable.getEntry("Update Count").setNumber(navx.getUpdateCount());

		final NetworkTable positionTable = targetTable.getSubTable("position");
		positionTable.getEntry("X Displacement").setNumber(navx.getDisplacementX());
		positionTable.getEntry("Y Displacement").setNumber(navx.getDisplacementY());
		positionTable.getEntry("Z Displacement").setNumber(navx.getDisplacementZ());
		positionTable.getEntry("Altitude").setNumber(navx.getAltitude());
		positionTable.getEntry("X Accel").setNumber(navx.getRawAccelX());
		positionTable.getEntry("Y Accel").setNumber(navx.getRawAccelY());
		positionTable.getEntry("Z Accel").setNumber(navx.getRawAccelZ());

		final NetworkTable rotationTable = targetTable.getSubTable("rotation");
		rotationTable.getEntry("Yaw").setNumber(navx.getYaw());
		rotationTable.getEntry("Pitch").setNumber(navx.getPitch());
		rotationTable.getEntry("Roll").setNumber(navx.getRoll());
		rotationTable.getEntry("Pitch Accel").setNumber(navx.getRawGyroX());
		rotationTable.getEntry("Yaw Accel").setNumber(navx.getRawGyroY());
		rotationTable.getEntry("Roll Accel").setNumber(navx.getRawGyroZ());
		rotationTable.getEntry("Quat W").setNumber(navx.getQuaternionW());
		rotationTable.getEntry("Quat X").setNumber(navx.getQuaternionX());
		rotationTable.getEntry("Quat Y").setNumber(navx.getQuaternionY());
		rotationTable.getEntry("Quat Z").setNumber(navx.getQuaternionZ());








		return true;
	}

}
