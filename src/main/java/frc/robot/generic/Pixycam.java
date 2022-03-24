package frc.robot.generic;

import io.github.pseudoresonance.pixy2api.Pixy2;
import io.github.pseudoresonance.pixy2api.Pixy2CCC;
import io.github.pseudoresonance.pixy2api.links.SPILink;
import lombok.SneakyThrows;
import lombok.Value;

import java.util.concurrent.Semaphore;
import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;


import static io.github.pseudoresonance.pixy2api.Pixy2.*;



public class Pixycam extends Thread {

	boolean isRunning = true;

	private Pixy2 pixycam;
	private SPILink pixySPI = new SPILink();

	private final Semaphore cargoSearchPermit = new Semaphore(1);
	private final PixyCargo[] NO_CARGO = new PixyCargo[0];
	private AtomicReference<PixyCargo[]> pixyCargos = new AtomicReference<>(NO_CARGO);

	String status = "";

	@Override @SneakyThrows
	public void run() {
		try {
			int retc = 0;
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
			//pixycam.setLamp(0x01);
			Pixy2CCC ccc = pixycam.getCCC();
			while (isRunning) {
				//If nobody has seen our old data,
				//We don't have permission to get new data
				cargoSearchPermit.acquire();
				int blockCount = ccc.getBlocks(true);
				switch (blockCount) {
					case 0:
						//I'm happyish
						status = "PIXY RUN: No Cargo Found";
						//Comment out to stop spam
						System.out.println("PIXY RUN: No Cargo Found");
						break;
					case PIXY_RESULT_ERROR:
						//I'm not happy
						status = "PIXY RUN: General Error";
						System.out.println("PIXY RUN: General Error");
						return;
					case PIXY_RESULT_BUSY:
						//I'm not happy
						status = "PIXY RUN: Busy Error";
						System.out.println("PIXY RUN: Busy Error");
						return;
					case PIXY_RESULT_CHECKSUM_ERROR:
						//I'm not happy
						status = "PIXY RUN: Checksum Error";
						System.out.println("PIXY RUN: Checksum Error");
						return;
					case PIXY_RESULT_TIMEOUT:
						//I'm not happy
						status = "PIXY RUN: Timeout Error";
						System.out.println("PIXY RUN: Timeout Error");
						return;
					case PIXY_RESULT_BUTTON_OVERRIDE:
						//I'm not happy
						status = "PIXY RUN: Button Override Error";
						System.out.println("PIXY RUN: Button Override Error");
						return;
					case PIXY_RESULT_PROG_CHANGING:
						//I'm not happy
						status = "PIXY RUN: Program Change Error";
						System.out.println("PIXY RUN: Program Change Error");
						return;
					default:
						//I'm happy
						status = "PIXY RUN: Target sighted";
						System.out.println(status);

						PixyCargo[] cargosFound = new PixyCargo[blockCount];
						ArrayList<Pixy2CCC.Block> blocksFound = ccc.getBlockCache();

						int i = 0;
						for (Pixy2CCC.Block block : blocksFound) {
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
		} catch (Exception e) {
			while (true) {
				e.printStackTrace();
				try {
					Thread.sleep(1000);
				} catch (InterruptedException e1) {}
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
		return cargoList[0];
	}

	@Value
	public static class PixyCargo {
		static final int FRAME_WIDTH = 320;
		static final int FRAME_HEIGHT = 200;
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

		public String toString() {
			String shortColor = (color == PixyCargoColor.RED) ? "RED" : "BLU";
			return  "offset=" + (x-157) +
					" age=" + age +
					" clr=" + shortColor +
					" area=" + (w*h) +
					" asp.=" + (w/h) +
					" x=" + x +
					" y=" + y +
					" w=" + w +
					" h=" + h;

		}

		public double getProportionalOffsetX(){
			return (((double) x) / FRAME_WIDTH) * 2 - 1;
		}
		public double getProportionalOffsetY(){
			return (((double) y) / FRAME_HEIGHT) * 2 - 1;
		}
	}
}
