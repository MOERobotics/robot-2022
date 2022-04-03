package frc.robot.diagnostics;

import edu.wpi.first.networktables.*;
import lombok.SneakyThrows;

import java.text.DateFormat;
import java.time.Instant;
import java.time.format.DateTimeFormatter;
import java.time.temporal.TemporalAccessor;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.ConcurrentSkipListSet;
import java.util.concurrent.atomic.AtomicBoolean;

public class DiagnosticsController extends Thread {
	public static final int UPDATE_RATE_MS      = 1000;
	public static final int UPDATE_COMPONENT_MS =    2;
	public static final DateTimeFormatter timestampFormat = DateTimeFormatter.ofPattern("HH:mm:ss.SSSS");
	public static final DiagnosticsController instance = new DiagnosticsController();

	private final ConcurrentLinkedQueue<DiagnosticsModule> loadedModules = new ConcurrentLinkedQueue<>();
	private final NetworkTableInstance nt          = NetworkTableInstance.getDefault();
	private final NetworkTable         root        = nt.getTable("MOE_Diagnostics");
	private final NetworkTableEntry    lastUpdated = root.getEntry("LastUpdate");
	private final AtomicBoolean        doShutdown  = new AtomicBoolean(false);
	private DiagnosticsController() {
		lastUpdated.setString(getTimestamp());
	}

	public static String getTimestamp() {
		return getTimestamp(java.time.LocalTime.now());
	}
	public static String getTimestamp(TemporalAccessor time) {
		return timestampFormat.format(time);
	}

	public void _addModule(String name, DiagnosticsLoader loader) {
		loadedModules.add(new DiagnosticsModule(
			root,
			name,
			loader
		));
	}
	public static void addModule(String name, DiagnosticsLoader loader) {
		instance._addModule(name,loader);
	}

	@SuppressWarnings("BusyWait")
	@Override @SneakyThrows
	public void run() {
		this.setName("DiagnosticsController");
		while (doShutdown.get() == false) {
			for (DiagnosticsModule module : loadedModules) {
				module.update();
				Thread.sleep(UPDATE_COMPONENT_MS);
			}
			lastUpdated.setString(getTimestamp());
			Thread.sleep(UPDATE_RATE_MS);
		}
	}

}
