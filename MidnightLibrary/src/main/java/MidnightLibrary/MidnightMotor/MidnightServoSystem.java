package MidnightLibrary.MidnightMotor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;
import java.util.List;

import MidnightLibrary.MidnightResources.MidnightHardware;
import MidnightLibrary.MidnightSensors.MidnightLimitSwitch;

/**
 * Created by Archish on 10/10/17.
 */

public class MidnightServoSystem implements MidnightHardware {
    public final MidnightServo servo1, servo2, servo3, servo4;
    public final List<MidnightServo> servos;
    private int offset;
    private double position;

    public MidnightServoSystem(String name1, String name2, HardwareMap hardwareMap) {
        this(new MidnightServo(name1, hardwareMap), new MidnightServo(name2, hardwareMap));
    }

    public MidnightServoSystem(String name1, String name2, String name3, HardwareMap hardwareMap) {
        this(new MidnightServo(name1, hardwareMap), new MidnightServo(name2, hardwareMap), new MidnightServo(name3, hardwareMap));
    }

    public MidnightServoSystem(String name1, String name2, String name3, String name4, HardwareMap hardwareMap) {
        this(new MidnightServo(name1, hardwareMap), new MidnightServo(name2, hardwareMap), new MidnightServo(name3, hardwareMap), new MidnightServo(name4, hardwareMap));
    }

    public MidnightServoSystem(String name1, Servo.Direction direction1,
                               String name2, Servo.Direction direction2, HardwareMap hardwareMap) {
        this(new MidnightServo(name1, direction1, hardwareMap),
                new MidnightServo(name2, direction2, hardwareMap));
    }

    public MidnightServoSystem(String name1, Servo.Direction direction1,
                               String name2, Servo.Direction direction2,
                               String name3, Servo.Direction direction3, HardwareMap hardwareMap) {
        this(new MidnightServo(name1, direction1, hardwareMap),
                new MidnightServo(name2, direction2, hardwareMap),
                new MidnightServo(name3, direction3, hardwareMap));
    }

    public MidnightServoSystem(String name1, Servo.Direction direction1,
                               String name2, Servo.Direction direction2,
                               String name3, Servo.Direction direction3,
                               String name4, Servo.Direction direction4, HardwareMap hardwareMap) {
        this(new MidnightServo(name1, direction1, hardwareMap),
                new MidnightServo(name2, direction2, hardwareMap),
                new MidnightServo(name3, direction3, hardwareMap),
                new MidnightServo(name4, direction4, hardwareMap));
    }

    public MidnightServoSystem(MidnightServo one, MidnightServo two) {
        servo1 = one;
        servo2 = two;
        servo3 = null;
        servo4 = null;
        servos = Arrays.asList(servo1, servo2);
    }

    public MidnightServoSystem(MidnightServo one, MidnightServo two, MidnightServo three) {
        servo1 = one;
        servo2 = two;
        servo3 = three;
        servo4 = null;
        servos = Arrays.asList(servo1, servo2, servo3);
    }

    public MidnightServoSystem(MidnightServo one, MidnightServo two, MidnightServo three, MidnightServo four) {
        servo1 = one;
        servo2 = two;
        servo3 = three;
        servo4 = four;
        servos = Arrays.asList(servo1, servo2, servo3, servo4);
    }

    public double getPosition() {
        return position;
    }

    public void setPosition(double position) {
        this.position = position;
        int i = 0;
        for (MidnightServo s : servos) {
            s.setPosition(position + (i * offset));
        }
    }

    public MidnightServoSystem setLimits(MidnightLimitSwitch min, MidnightLimitSwitch max) {
        for (MidnightServo s : servos) s.setLimits(min, max);
        return this;
    }

    public void setOffset(int offset) {
        this.offset = offset;
    }

    public void scaleRange(double min, double max) {
        for (MidnightServo servo : servos) {
            servo.scaleRange(min, max);
        }
    }

    public void setServo1Pos(double position) {
        servo1.setPosition(position);
    }

    public void setServo2Pos(double position) {
        servo2.setPosition(position);
    }

    @Override
    public String getName() {
        return "SERVO SYSTEM";
    }

    @Override
    public String[] getDash() {
        return new String[0];
    }
}
