package MidnightLibrary.MidnightSensors;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import MidnightLibrary.MidnightResources.MidnightHardware;
import MidnightLibrary.MidnightResources.MidnightUtils;


/**
 * Created by Archish on 10/28/16.
 */

public class MidnightODS implements MidnightHardware {
    private OpticalDistanceSensor ods;
    private String nameODSSensor;

    public MidnightODS(String name, HardwareMap hardwareMap){
        this.nameODSSensor = name;
        ods = hardwareMap.opticalDistanceSensor.get(name);
    }
    public void enableLED() {
        ods.enableLed(true);
    }
    public void disableLED() {
        ods.enableLed(false);
    }
    public double lightDetected () {
        return ods.getLightDetected();
    }
    public double rawLight () {
        return ods.getRawLightDetected();
    }
    public boolean isWhite () {
        return lightDetected() <= MidnightUtils.ODS_WHITE;
    }
    public boolean isBlack () {
        return lightDetected() >= MidnightUtils.ODS_BLACK;
    }
    public String getName() {
        return nameODSSensor;
    }
    public String[] getDash() {
        return new String[]{
                "Light Detected" + lightDetected(),
                "Raw Ligh" + rawLight(),
                "is Black:" + isBlack(),
                "IS White:" + isWhite()
        };
    }
}
