package frc.robot.generic;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.links.SPILink;
import lombok.SneakyThrows;
import lombok.Value;

import java.util.concurrent.Semaphore;
import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;


import static edu.wpi.first.wpilibj.DriverStation.getAlliance;
import static io.github.pseudoresonance.pixy2api.Pixy2.*;



public class Pixycam extends Thread {

	boolean isRunning = true;

	private Pixy2 pixycam;
	private SPILink pixySPI = new SPILink();

	private final Semaphore cargoSearchPermit = new Semaphore(1);
	private final PixyCargo[] NO_CARGO = new PixyCargo[0];
	private AtomicReference<PixyCargo[]> pixyCargos = new AtomicReference<>(NO_CARGO);

	String status = "";
	String msg = "Init...";

	double generalErrorCount = 0;

	Timer timer;

	@Override @SneakyThrows
	public void run() {
		int retc = 0;
		timer = new Timer();
		pixycam = Pixy2.createInstance(pixySPI);
		retc = pixycam.init();
		switch (retc) {
			case PIXY_RESULT_OK:
				//I'm happy
				status = "PIXY INIT: Success!";
				System.out.println("PIXY INIT: Success!");
				break;
			default:
			case PIXY_RESULT_ERROR:
				//I'm not happy
				status = "PIXY INIT: General Error";
				System.out.println("PIXY INIT: General Error");
				return;
			case PIXY_RESULT_BUSY:
				//I'm not happy
				status = "PIXY INIT: Busy Error";
				System.out.println("PIXY INIT: Busy Error");
				return;
			case PIXY_RESULT_CHECKSUM_ERROR:
				//I'm not happy
				status = "PIXY INIT: Checksum Error";
				System.out.println("PIXY INIT: Checksum Error");
				return;
			case PIXY_RESULT_TIMEOUT:
				//I'm not happy
				status = "PIXY INIT: Timeout Error";
				System.out.println("PIXY INIT: Timeout Error");
				return;
			case PIXY_RESULT_BUTTON_OVERRIDE:
				//I'm not happy
				status = "PIXY INIT: Button Override Error";
				System.out.println("PIXY INIT: Button Override Error");
				return;
			case PIXY_RESULT_PROG_CHANGING:
				//I'm not happy
				status = "PIXY INIT: Program Change Error";
				System.out.println("PIXY INIT: Program Change Error");
				return;
		}
		pixycam.setLamp((byte)1, (byte)0);
		Pixy2CCC ccc = pixycam.getCCC();
		while(isRunning) {
			//If nobody has seen our old data,
			//We don't have permission to get new data

			cargoSearchPermit.acquire();
			//timer.delay(0.02);

			int blockCount = ccc.getBlocks(true);
			switch (blockCount) {
				case 0:
					//I'm happyish
					status = "PIXY RUN: No Cargo Found";
					//Comment out to stop spam
					//System.out.println("PIXY RUN: No Cargo Found");
					break;
				case PIXY_RESULT_ERROR:
					//I'm not happy
					status = "PIXY RUN: General Error";
					System.out.println("PIXY RUN: General Error");

					generalErrorCount++;
					//return;
					break;
				case PIXY_RESULT_BUSY:
					//I'm not happy
					status = "PIXY RUN: Busy Error";
					System.out.println("PIXY RUN: Busy Error");

					generalErrorCount += 0.01;
					//return;
					break;
				case PIXY_RESULT_CHECKSUM_ERROR:
					//I'm not happy
					status = "PIXY RUN: Checksum Error";
					System.out.println("PIXY RUN: Checksum Error");

					generalErrorCount += 0.0001;
					//return;
					break;
				case PIXY_RESULT_TIMEOUT:
					//I'm not happy
					status = "PIXY RUN: Timeout Error";
					System.out.println("PIXY RUN: Timeout Error");

					generalErrorCount += 0.0001;
					//return;
					break;
				case PIXY_RESULT_BUTTON_OVERRIDE:
					//I'm not happy
					status = "PIXY RUN: Button Override Error";
					System.out.println("PIXY RUN: Button Override Error");
					//return;
					break;
				case PIXY_RESULT_PROG_CHANGING:
					//I'm not happy
					status = "PIXY RUN: Program Change Error";
					System.out.println("PIXY RUN: Program Change Error");
					//return;
					break;
				default:
					//I'm happy
					status = "PIXY RUN: Target sighted";
					System.out.println(status);

					PixyCargo[] cargosFound = new PixyCargo[blockCount];
					ArrayList<Pixy2CCC.Block> blocksFound = ccc.getBlockCache();

					int i = 0;
					for(Pixy2CCC.Block block : blocksFound){
						PixyCargo pcargo = new PixyCargo(
							block.getX(),
							block.getY(),
							block.getWidth(),
							block.getHeight(),
							block.getAge(),
							block.getIndex(),
							//color
							(block.getSignature() == 1)
								? PixyCargo.PixyCargoColor.RED
								: PixyCargo.PixyCargoColor.BLUE
						);

						cargosFound[i++] = pcargo;
					}

					//DO NOT SET THIS UNTIL WE'RE DONE LOOKING
					this.pixyCargos.set(cargosFound);
					break;
			}
		}
	}

	public PixyCargo[] getCargo(){
		return this.getCargo(false);
	}

	//Refresh = true only if call is from robot-periodic, controls 50 semaphore sets/second
	public PixyCargo[] getCargo(boolean refresh) {
		PixyCargo[] result = pixyCargos.get();

		//Give us a permit to conduct a new cargo search
		if(refresh) cargoSearchPermit.release();
		return result;
	}




	public String getStatus(){
		return status;
	}


	public PixyCargo identifyClosestCargo(){
		return identifyClosestCargo(getCargo(false));
	}

	public PixyCargo identifyClosestCargo(PixyCargo[] cargoList){
		if(cargoList.length == 0) return null;
		double[] scores = new double[cargoList.length];
		for (int i = 0; i < cargoList.length; i++) {
			PixyCargo cargo = cargoList[i];
			//double aspectRatio = cargo.getW() / cargo.getH();
			double bindingDimension = Math.min(cargo.getW(), cargo.getH());
			scores[i] = bindingDimension * bindingDimension;
			scores[i] *= (Math.max(cargo.getAge(), 10)/5.0 + 1)/3;
			if(!cargo.cargoMatchesAlliance()) scores[i] = 0;
		}
		int maxIndex = 0;
		double maxScore = scores[0];
		for (int i = 1; i < cargoList.length; i++) {
			if(scores[i] > maxScore){
				maxIndex = i;
				maxScore = scores[i];
			}
		}
		return cargoList[maxIndex];
	}

	@Value
	public static class PixyCargo {
		static final int FRAME_WIDTH = 315;
		static final int FRAME_HEIGHT = 208;
		int x;
		int y;
		int w;
		int h;
		int age;
		int id;
		PixyCargoColor color;

		public enum PixyCargoColor {
			RED,
			BLUE,
		}

		public static PixyCargoColor getAlliance() {
			AllianceStationID allianceStationID = HAL.getAllianceStation();
			if (allianceStationID == null) {
			      return PixyCargoColor.RED;
			}
			switch (allianceStationID) {
			     case Red1:
			     case Red2:
			     case Red3:
			         return PixyCargoColor.RED;
				 case Blue1:
				 case Blue2:
				 case Blue3:
				      return PixyCargoColor.BLUE;

			    default:
			          return PixyCargoColor.RED;
			}
		}

		public boolean cargoMatchesAlliance(){
			return this.color == getAlliance();
		}

		public String toString() {

			String shortColor = (color == getAlliance()) ? "RED" : "BLU";
			return  "offset=" + (y-104) +
					" age=" + age +
					" clr=" + shortColor +
					" area=" + (w*h) +
					" asp.=" + (w/h) +
					" x=" + x +
					" y=" + y +
					" w=" + w +
					" h=" + h +
					" id=" + id;

		}

		public double getProportionalOffsetX(){
			return (((double) x) / FRAME_WIDTH) * 2 - 1;
		}
		public double getProportionalOffsetY(){
			return (((double) y) / FRAME_HEIGHT) * 2 - 1;
		}
	}

	public double getGeneralErrorCount(){
		return generalErrorCount;
	}

}
