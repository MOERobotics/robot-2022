package frc.robot.diagnostics;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;

import static frc.robot.diagnostics.DiagnosticsController.getTimestamp;

public class DiagnosticsModule {
	public final DiagnosticsLoader loader;
	public final NetworkTable table;
	public final NetworkTableEntry lastUpdated;
	public DiagnosticsModule(
		NetworkTable diagnosticsRoot,
		String moduleName,
		DiagnosticsLoader moduleReader
	) {
		this.loader = moduleReader;
		this.table = diagnosticsRoot.getSubTable(moduleName);
		this.lastUpdated = table.getEntry("LastUpdated");
		this.lastUpdated.setString(getTimestamp());
	}

	public boolean update() {
		boolean successfulUpdate = loader.loadDiagnostics(table);
		if (successfulUpdate) this.lastUpdated.setString(getTimestamp());
		return successfulUpdate;
	}
}
