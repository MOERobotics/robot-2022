package frc.robot.generic;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import lombok.SneakyThrows;
import lombok.Value;

public interface GenericPixycam extends Runnable {

    public void run();

    public Pixycam.PixyCargo[] getCargo();

    //Refresh = true only if call is from robot-periodic, controls 50 semaphore sets/second
    public Pixycam.PixyCargo[] getCargo(boolean refresh);

    public String getStatus();

    public PixyCargo identifyClosestCargo();
    public PixyCargo identifyClosestCargo(PixyCargo[] cargoList);

    public double getGeneralErrorCount();

    public void start();
    public void stop();


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

}
