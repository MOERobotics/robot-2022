package frc.robot.diagnostics;

import edu.wpi.first.networktables.NetworkTable;

public interface DiagnosticsLoader {
	public boolean loadDiagnostics(NetworkTable targetTable);
}
