package frc.robot.generic;

import edu.wpi.first.wpilibj.SerialPort;

import java.nio.charset.StandardCharsets;
import java.util.HashMap;

public class Lidar extends Thread {


	//Ex: " 1234-5678  "
	//where 1234 is the Network number (id)
	//  and 5678 is the distance in arbitrary units

	public HashMap<Integer, Integer> distances = new HashMap<>();
	public final Object distanceLock = new Object();
	public int getDistance(int id) {
		synchronized (distanceLock) {return distances.getOrDefault(id,-1);}
	}

	@Override public void run() {

		SerialPort lidarSerialPort = null;
		lidarSerialPort = new SerialPort(
			9600,
			SerialPort.Port.kMXP,
			8,
			SerialPort.Parity.kNone,
			SerialPort.StopBits.kOne
		);

		//"-8192 ";

		StringBuilder partialString = new StringBuilder();
		while(true) {

			byte[] newByte = lidarSerialPort.read(1);
			if (newByte.length == 0) {/*File closed*/}
			char characterRead = (char)newByte[0];


			switch (characterRead) {
				case '\n':
				case '\r':
				case ' ':
					break;
				default:
					partialString.append(characterRead);
			}

			if (characterRead == ' '){
				String e = partialString.toString();
				partialString.delete(0,999);

				if(e.length()>1 && e.contains("-")){
					if(e.indexOf("-") == 0)
						e = " " + e;
					//System.out.printf("Lidar '%s'", e);
					String[] ID = e.split("-");
					int id = Integer.parseInt(ID[0]);
					int length = Integer.parseInt(ID[1]);
					synchronized (distanceLock) { distances.put(id,length);}
				}
			}

		}
	}
}
