package frc.robot.diagnostics;

import edu.wpi.first.networktables.*;

import java.text.DateFormat;
import java.time.Instant;
import java.time.format.DateTimeFormatter;
import java.time.temporal.TemporalAccessor;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.ConcurrentSkipListSet;

public class DiagnosticsController extends Thread {
	public static final int UPDATE_RATE_MS      = 1000;
	public static final int UPDATE_COMPONENT_MS =    1;
	public static final DateTimeFormatter timestampFormat = DateTimeFormatter.ofPattern("HH:mm:ss.SSSS");
	public static final DiagnosticsController instance = new DiagnosticsController();

	private final ConcurrentLinkedQueue<DiagnosticsModule> loadedModules = new ConcurrentLinkedQueue<>();
	private final NetworkTableInstance nt          = NetworkTableInstance.getDefault();
	private final NetworkTable         root        = nt.getTable("MOE_Diagnostics");
	private final NetworkTableEntry    lastUpdated = root.getEntry("LastUpdate");
	private DiagnosticsController() {
		lastUpdated.setString(getTimestamp());
	}

	public static String getTimestamp() {
		return timestampFormat.format(Instant.now());
	}
	public static String getTimestamp(TemporalAccessor time) {
		return timestampFormat.format(time);
	}

	public static

}
