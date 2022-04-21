package frc.robot.generic;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class WebcamPixy implements GenericPixycam {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("pixy");
    NetworkTableEntry bestredx = table.getEntry("bestRed");
    NetworkTableEntry bestbluex = table.getEntry("bestBlue");


    @Override
    public PixyCargo[] getCargo() {
        PixyCargo[] cargos = new PixyCargo[1];
        //bogus neutral cargo
        cargos[0] = new PixyCargo(0, 0, 10, 10, 1, 1, PixyCargo.getAlliance());;

        double seenXVal = 2; //out of bounds so basically NaN, ignored
        if(PixyCargo.getAlliance() == PixyCargo.PixyCargoColor.RED){
            seenXVal = bestredx.getDouble(2);
        }else{
            seenXVal = bestbluex.getDouble(2);
        }
        if(seenXVal > -1 && seenXVal < 1){
            int rawX = (int)Math.round(seenXVal * PixyCargo.FRAME_HEIGHT);
            PixyCargo cargo = new PixyCargo(0, rawX, 10, 10, 1, 1, PixyCargo.getAlliance());
            cargos[0] = cargo;
        }

        return cargos;
    }

    @Override
    public PixyCargo identifyClosestCargo(PixyCargo[] cargoList) {
        return getCargo()[0];
    }


    //refresh doesn't mean anything in webcam implementation
    @Override
    public PixyCargo[] getCargo(boolean refresh) {
        return getCargo();
    }

    @Override
    public PixyCargo identifyClosestCargo() {
        return identifyClosestCargo(getCargo());
    }

    @Override
    public String getStatus() {
        return "Using webcam";
    }

    @Override
    public double getGeneralErrorCount() {
        return 0;
    }

    @Override
    public void start() {
        //do nothing
    }

    @Override
    public void stop() {
        //do nothing
    }
}
