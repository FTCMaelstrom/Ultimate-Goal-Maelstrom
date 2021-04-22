package MidnightLibrary.MidnightSensors;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import MidnightLibrary.MidnightAuxiliary.MidnightHardware;

public class MidnightTouchSensor implements MidnightHardware {
    private final DigitalChannel touchSensor;
    private final String nameTouchSensor;

    public MidnightTouchSensor(String name, HardwareMap hardwareMap) {
        this.nameTouchSensor = name;
        touchSensor = hardwareMap.digitalChannel.get(name);
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean isPressed() {
        return touchSensor.getState();
    }

    public String getName() {
        return nameTouchSensor;
    }

    public String[] getDash() {
        return new String[]{
                "Pressed: " + isPressed()
        };
    }


    public boolean stop() {
        return !isPressed();
    }
}