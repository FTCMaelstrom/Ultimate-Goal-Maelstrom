package Library17822.MidnightSensors;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Library17822.MinightResources.MidnightHelpers.MidnightHardware;

public class MidnightTouchSensor implements MidnightHardware {
    private DigitalChannel touchSensor;
    private String nameTouchSensor;
    public MidnightTouchSensor(String name, HardwareMap hardwareMap){
        this.nameTouchSensor = name;
        touchSensor = hardwareMap.digitalChannel.get(name);
        touchSensor.setMode(DigitalChannel.Mode.INPUT);
    }
    public boolean isPressed () {
        return touchSensor.getState();
    }

    public String getName() {
        return nameTouchSensor;
    }
    public String[] getDash() {
        return new String[]{
                "Pressed: " + Boolean.toString(isPressed())
        };
    }


    public boolean stop() {
        return !isPressed();
    }
}