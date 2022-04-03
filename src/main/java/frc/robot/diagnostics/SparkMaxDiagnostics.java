package frc.robot.diagnostics;

import com.revrobotics.*;
import edu.wpi.first.networktables.NetworkTable;
import lombok.Data;
import lombok.Value;

@Value
public class SparkMaxDiagnostics implements DiagnosticsLoader {
	CANSparkMax spark;
	@Override
	public boolean loadDiagnostics(NetworkTable targetTable) {
		if (spark.getFirmwareVersion() <1) return false;
		targetTable.getEntry("Can ID"           ).setNumber  (spark.getDeviceId        ());
		targetTable.getEntry("Applied Output"   ).setNumber  (spark.getAppliedOutput   ());
		targetTable.getEntry("Bus Voltage"      ).setNumber  (spark.getBusVoltage      ());
		targetTable.getEntry("Temperature"      ).setNumber  (spark.getMotorTemperature());
		targetTable.getEntry("Output Current"   ).setNumber  (spark.getOutputCurrent   ());
		targetTable.getEntry("Firmware Version" ).setNumber  (spark.getFirmwareVersion ());
		targetTable.getEntry("Output"           ).setNumber  (spark.get                ());
		targetTable.getEntry("Inversion"        ).setBoolean (spark.getInverted        ());
		targetTable.getEntry("Idle mode"        ).setString  (spark.getIdleMode().toString());

		final RelativeEncoder sparkEncoder = spark.getEncoder();
		final NetworkTable encoderTable = targetTable.getSubTable("encoder");
		encoderTable.getEntry("Position"        ).setNumber  (sparkEncoder.getPosition ());
		encoderTable.getEntry("Velocity"        ).setNumber  (sparkEncoder.getVelocity ());
		encoderTable.getEntry("Inversion"       ).setBoolean (sparkEncoder.getInverted ());

		final SparkMaxPIDController pidController = spark.getPIDController();
		final NetworkTable pidTable = targetTable.getSubTable("pid");
		pidTable.getEntry("Position"        ).setNumber  (pidController.getP ());
		pidTable.getEntry("Integral"        ).setNumber  (pidController.getI ());
		pidTable.getEntry("Derivative"      ).setNumber  (pidController.getD ());
		pidTable.getEntry("Feed Forward"    ).setNumber  (pidController.getFF());
		pidTable.getEntry("IZone"           ).setNumber  (pidController.getIZone ());
		pidTable.getEntry("IMaxAcc"         ).setNumber  (pidController.getIMaxAccum (0));
		pidTable.getEntry("DFilter"         ).setNumber  (pidController.getDFilter (0));
		pidTable.getEntry("Output Min"      ).setNumber  (pidController.getOutputMin ());
		pidTable.getEntry("Output Max"      ).setNumber  (pidController.getOutputMax ());


		return true;
	}
}
