package frc.robot.generic;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class WebcamPixy implements GenericPixycam {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("pixy");
    NetworkTableEntry red = table.getEntry("RED");



    @Override
    public PixyCargo[] getCargo() {
        return new PixyCargo[0];
    }

    @Override
    public PixyCargo identifyClosestCargo(PixyCargo[] cargoList) {
        return null;
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
