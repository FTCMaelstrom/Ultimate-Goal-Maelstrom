package MidnightLibrary.MidnightMovement;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import MidnightLibrary.MidnightAuxiliary.MidnightHardware;
import MidnightLibrary.MidnightSensors.MidnightLimitSwitch;

import static MidnightLibrary.MidnightAuxiliary.MidnightUtils.opModeIsActive;

/**
 * Created by Archish on 10/28/16.
 */

public class MidnightServo implements MidnightHardware {
    private final Servo servo;
    private final String name;
    private final double max = 1;
    private final double min = 0;
    private MidnightLimitSwitch limMin, limMax;
    private boolean limDetection;
    private double position;
    private boolean prevState = false;
    private boolean taskState = false;
    private boolean positionControlState = false;


    public MidnightServo(String name, HardwareMap hardwareMap) {
        this.name = name;
        servo = hardwareMap.servo.get(name);
    }

    public MidnightServo(String name, Servo.Direction direction, HardwareMap hardwareMap) {
        this.name = name;
        servo = hardwareMap.servo.get(name);
        servo.setDirection(direction);
    }

    public void setDirection(Servo.Direction direction) {
        servo.setDirection(direction);
    }

    public void setLimits(MidnightLimitSwitch min, MidnightLimitSwitch max) {
        limMin = min;
        limMax = max;
        limDetection = true;
    }

    private boolean limitPressed() {
        if (limDetection) return limMin.isPressed() || limMax.isPressed();
        return false;
    }

    public double getPosition() {
        return servo.getPosition();
    }

    public void setPosition(double position) {
        this.position = position;
        servo.setPosition(position);
    }

    public void scaleRange(double min, double max) {
        servo.scaleRange(min, max);
    }

    public void sleep(int time) throws InterruptedException {
        servo.wait(time);
    }

    public void toggle(boolean button, double pos1, double pos2) {
        boolean currState = false;

        if (button) currState = true;
        else if (prevState) taskState = !taskState;

        prevState = currState;

        if (taskState) setPosition(pos1);
        else setPosition(pos2);
    }

    public void toggle(boolean button) {
        toggle(button, 0, 1);
    }

    public void setPositionControlState(boolean positionControlState) {
        this.positionControlState = positionControlState;
    }

    public void startPositionControl() {
        positionControlState = true;
        Runnable positionControl = () -> {
            while (opModeIsActive() && positionControlState) setPosition(position);
        };
        Thread positionThread = new Thread(positionControl);
        positionThread.start();
    }

    public String getName() {
        return name;
    }

    public String[] getDash() {
        return new String[]{
                "Current Position:" + servo.getPosition()
        };
    }
}