package MidnightLibrary.MidnightMovement;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import MidnightLibrary.MidnightSensors.MidnightLimitSwitch;
import MidnightLibrary.MidnightResources.MidnightHardware;

/**
 * Created by Archish on 11/4/16.
 */

public class MidnightCRServo implements MidnightHardware {
    private CRServo servo;
    private String nameCr_Servo;
    private MidnightLimitSwitch min, max = null;
    private boolean limitDetection;
    public MidnightCRServo(String name, HardwareMap hardwareMap){
        this.nameCr_Servo = name;
        servo = hardwareMap.crservo.get(name);
        limitDetection = false;
    }
    public MidnightCRServo(String name, CRServo.Direction direction, HardwareMap hardwareMap){
        this.nameCr_Servo = name;
        servo = hardwareMap.crservo.get(name);
        servo.setDirection(direction);
        limitDetection = false;
    }
    public void setLimits(MidnightLimitSwitch min, MidnightLimitSwitch max){
        this.min = min; this.max = max;
        limitDetection = true;
    }
    public void setLimit(MidnightLimitSwitch min) {
        this.min = min;
        this.max = null;
        limitDetection = false;
    }
    public void setPower (double power) {
        double motorPower = power;
        if (limitDetection) {
            if (min != null && min.getState() && power < 0 ||
                    max != null && max.getState() && power > 0)
                motorPower = 0;
        }
        servo.setPower(motorPower);
    }
    public void sleep (int time) throws InterruptedException {servo.wait(time);}
    public double getPower() {return servo.getPower();}
    public String getName() {return nameCr_Servo;}

    public String[] getDash() {
        return new String[]{
                "Current Power" + Double.toString(getPower())
        };
    }
}