package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lombok.SneakyThrows;
import lombok.Value;
import lombok.experimental.Helper;

import java.time.Duration;
import java.time.Instant;
import java.util.*;
import java.util.concurrent.Semaphore;
import java.util.concurrent.atomic.AtomicReference;

public class SmarterDashboard {

	private SmarterDashboard(){}

	@Value public static class SDValue {
		String key;
		SDValueType type;
		Object value;
	}

	public static enum SDValueType {
		NUMBER,
		STRING,
		BOOLEAN,
	}

	private static final Object lock = new Object();
	private static final HashMap<String,Object> knownValues = new HashMap<>();
	private static AtomicReference<Queue<SDValue>> toProcess = new AtomicReference<>(new LinkedList<>());
	private static Semaphore transmissionPermit = new Semaphore(1);
	private static Thread transmissionThreadHandle = new Thread(
		SmarterDashboard::transmissionThread,
		"kwestNTThread"
	);

	static {
		transmissionThreadHandle.start();
	}

	public static void transmit() {
		transmissionPermit.release();
	}

	@SneakyThrows private static void transmissionThread() {
		while (true) {
			Queue<SDValue> replacementQueue = new LinkedList<>();
			Queue<SDValue> toProcessLocal = toProcess.get();
			transmissionPermit.acquire();
			synchronized (lock) {
				toProcess.set(replacementQueue);
			}
			Instant start = Instant.now();
			for (SDValue val : toProcessLocal) {
				switch (val.type) {
					case NUMBER : SmartDashboard.putNumber (val.key, (Double ) val.value); break;
					case STRING : SmartDashboard.putString (val.key, (String ) val.value); break;
					case BOOLEAN: SmartDashboard.putBoolean(val.key, (Boolean) val.value); break;
				}
			}
			Instant end = Instant.now();
			double duration = (1e-9)*Duration.between(start,end).toNanos();

			SmartDashboard.putString("KwestNTThread", String.format("%6f",duration));

		}
	}

	public static void sendNumber(String key, double value) {
		if (
			knownValues.containsKey(key) &&
			knownValues.get(key).equals(value)
		) return;
		knownValues.put(key,value);
		SDValue valBox = new SDValue(key,SDValueType.NUMBER,value);
		synchronized (lock) {
			toProcess.get().add(valBox);
		}
	}

	public static void sendString(String key, String value) {
		if (
			knownValues.containsKey(key) &&
				knownValues.get(key).equals(value)
		) return;
		knownValues.put(key,value);
		SDValue valBox = new SDValue(key,SDValueType.STRING,value);
		synchronized (lock) {
			toProcess.get().add(valBox);
		}
	}

	public static void sendBoolean(String key, boolean value) {
		if (
			knownValues.containsKey(key) &&
				knownValues.get(key).equals(value)
		) return;
		knownValues.put(key,value);
		SDValue valBox = new SDValue(key,SDValueType.BOOLEAN,value);
		synchronized (lock) {
			toProcess.get().add(valBox);
		}
	}

}
