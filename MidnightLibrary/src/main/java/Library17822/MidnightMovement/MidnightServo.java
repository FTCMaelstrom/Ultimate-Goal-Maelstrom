package Library17822.MidnightMovement;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import Library17822.MidnightSensors.MidnightLimitSwitch;
import Library17822.MidnightResources.MidnightHardware;

/**
 * Created by Archish on 10/28/16.
 */

public class MidnightServo implements MidnightHardware {
    private Servo servo;
    private String name;
    private double max = 1, min = 0;
    private MidnightLimitSwitch limMin, limMax;
    private boolean limDetection;
    private double adjustedPosition;


    public MidnightServo(String name, HardwareMap hardwareMap) {
        this.name = name;
        servo = hardwareMap.servo.get(name);
    }
    public MidnightServo(String name, Servo.Direction direction, HardwareMap hardwareMap){
        this.name = name;
        servo = hardwareMap.servo.get(name);
        servo.setDirection(direction);
    }
    public void setPosition (double position) {
        adjustedPosition = ((max - min) * position) + min;
        servo.setPosition(adjustedPosition);
    }
    public void setDirection(Servo.Direction direction) {
        servo.setDirection(direction);
    }
    public void setLimits (MidnightLimitSwitch min, MidnightLimitSwitch max){
        limMin = min; limMax = max;
        limDetection = true;
    }
    private boolean limitPressed () {
        if (limDetection) return  limMin.isPressed() || limMax.isPressed();
        return false;
    }
    public double getPosition () {
        return servo.getPosition();
    }
    public double getRawPosition() {
        return adjustedPosition;
    }
    public void setMax(double max){this.max = max;}
    public void setMin(double min){this.min = min;}
    public void scaleRange (double min, double max) {
        servo.scaleRange(min, max);
    }
    public void sleep (int time) throws InterruptedException {
        servo.wait(time);
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