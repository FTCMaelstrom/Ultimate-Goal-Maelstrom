package MidnightLibrary.MidnightSensors;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;

import MidnightLibrary.MidnightAuxiliary.MidnightHardware;

public class MidnightRangeSensor implements MidnightHardware {

    public static final int RANGE1_REG_START = 0x04;
    public static final int RANGE1_READ_LENGTH = 2;
    public I2cDevice RANGE1;
    public I2cDeviceSynch RANGE1Reader;
    byte[] range1Cache;
    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14);
    String nameRangeSensor;
    private int stopThresh = 0;

    public MidnightRangeSensor(String name, HardwareMap hardwareMap) {
        this.nameRangeSensor = name;
        RANGE1 = hardwareMap.i2cDevice.get("rangeSensor");
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();
    }

    public void setStopThresh(int thresh) {
        stopThresh = thresh;
    }

    public double Ultrasonic() {
        range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
        return range1Cache[0] & 0xFF;
    }

    public double ODS() {
        return range1Cache[1] & 0xFF;
    }

    public String getName() {
        return nameRangeSensor;
    }

    public String[] getDash() {
        return new String[]{
                "Raw Ultrasonic" + Ultrasonic()
        };
    }
}
