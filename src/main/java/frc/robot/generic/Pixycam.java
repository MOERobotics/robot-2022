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

	@Override @SneakyThrows
	public void run() {
		int retc = 0;
		pixycam = Pixy2.createInstance(pixySPI);
		retc = pixycam.init();
		switch (retc) {
			case PIXY_RESULT_OK:
				//I'm happy
				System.out.println("PIXY INIT: Success!");
				break;
			default:
			case PIXY_RESULT_ERROR:
				//I'm not happy
				System.out.println("PIXY INIT: General Error");
				return;
			case PIXY_RESULT_BUSY:
				//I'm not happy
				System.out.println("PIXY INIT: Busy Error");
				return;
			case PIXY_RESULT_CHECKSUM_ERROR:
				//I'm not happy
				System.out.println("PIXY INIT: Checksum Error");
				return;
			case PIXY_RESULT_TIMEOUT:
				//I'm not happy
				System.out.println("PIXY INIT: Timeout Error");
				return;
			case PIXY_RESULT_BUTTON_OVERRIDE:
				//I'm not happy
				System.out.println("PIXY INIT: Button Override Error");
				return;
			case PIXY_RESULT_PROG_CHANGING:
				//I'm not happy
				System.out.println("PIXY INIT: Program Change Error");
				return;
		}
		Pixy2CCC ccc = pixycam.getCCC();
		while(isRunning) {
			//If nobody has seen our old data,
			//We don't have permission to get new data
			cargoSearchPermit.acquire();
			int blockCount = ccc.getBlocks();
			switch (blockCount) {
				case 0:
					//I'm happyish
					//Todo: stop this from spamming
					System.out.println("PIXY INIT: No Cargo Found");
					break;
				case PIXY_RESULT_ERROR:
					//I'm not happy
					System.out.println("PIXY INIT: General Error");
					return;
				case PIXY_RESULT_BUSY:
					//I'm not happy
					System.out.println("PIXY INIT: Busy Error");
					return;
				case PIXY_RESULT_CHECKSUM_ERROR:
					//I'm not happy
					System.out.println("PIXY INIT: Checksum Error");
					return;
				case PIXY_RESULT_TIMEOUT:
					//I'm not happy
					System.out.println("PIXY INIT: Timeout Error");
					return;
				case PIXY_RESULT_BUTTON_OVERRIDE:
					//I'm not happy
					System.out.println("PIXY INIT: Button Override Error");
					return;
				case PIXY_RESULT_PROG_CHANGING:
					//I'm not happy
					System.out.println("PIXY INIT: Program Change Error");
					return;
				default:
					//I'm happy
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
								? PixyCargo.PixyCargoColor.PIXY_CARGO_RED
								: PixyCargo.PixyCargoColor.PIXY_CARGO_BLUE
						);

						cargosFound[i++] = pcargo;
					}
					//DO NOT SET THIS UNTIL WE'RE DONE LOOKING
					this.pixyCargos.set(cargosFound);
					break;
			}
		}
	}

	
	public PixyCargo[] getCargo() {
		PixyCargo[] result = pixyCargos.get();

		//Give us a permit to conduct a new cargo search
		cargoSearchPermit.release();
		return result;
	}



	@Value
	public static class PixyCargo {
		int x;
		int y;
		int w;
		int h;
		int age;
		int id;
		PixyCargoColor color;

		public static enum PixyCargoColor {
			PIXY_CARGO_RED,
			PIXY_CARGO_BLUE
		}

		public String toString(){
			return  "offset=" + (x-157) +
					" age=" + age +
					" id=" + id +
					" clr=" + color +
					" area=" + (w*h) +
					" x=" + x +
					" y=" + y +
					" w=" + w +
					" h=" + h;

		}
	}

}