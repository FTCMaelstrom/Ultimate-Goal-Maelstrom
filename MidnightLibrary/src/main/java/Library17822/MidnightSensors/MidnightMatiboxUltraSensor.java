package Library17822.MidnightSensors;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Library17822.MinightResources.MidnightHelpers.MidnightHardware;


/**
 * Created by Archish on 2/28/17.
 */

public class MidnightMatiboxUltraSensor implements MidnightHardware {
    private AnalogInput ds;
    private String nameDS;
    private int scale = 225;
    private double stopThresh = 60;
    public MidnightMatiboxUltraSensor(String name, HardwareMap hardwareMap){
            this.nameDS = name;
            ds = hardwareMap.get(AnalogInput.class, name);
        }
    public double getDistance() {return ds.getVoltage() * scale;}
    public void setStopThresh(int thresh) {
        stopThresh = thresh;
    }
    public String getName() {
            return nameDS;
        }
    public String[] getDash() {
            return new String[]{
                    "Distance " + getDistance()
            };
    }
}