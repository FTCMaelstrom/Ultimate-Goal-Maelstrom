package MidnightLibrary.MidnightMotor;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

import MidnightLibrary.MidnightResources.MidnightHardware;
import MidnightLibrary.MidnightSensors.MidnightLimitSwitch;

/**
 * Created by Archish on 10/9/17.
 */

public class MidnightCRServoSystem implements MidnightHardware {
    private final MidnightCRServo servo1, servo2, servo3, servo4;
    private final List<MidnightCRServo> servos;

    public MidnightCRServoSystem(String name1, String name2, HardwareMap hardwareMap) {
        this( new MidnightCRServo(name1, hardwareMap), new MidnightCRServo(name2, hardwareMap) );
    }

    public MidnightCRServoSystem(String name1, String name2, String name3, HardwareMap hardwareMap) {
        this( new MidnightCRServo(name1, hardwareMap), new MidnightCRServo(name2, hardwareMap), new MidnightCRServo(name3, hardwareMap) );
    }

    public MidnightCRServoSystem(String name1, String name2, String name3, String name4, HardwareMap hardwareMap) {
        this( new MidnightCRServo(name1, hardwareMap), new MidnightCRServo(name2, hardwareMap), new MidnightCRServo(name3, hardwareMap), new MidnightCRServo(name4, hardwareMap) );
    }

    public MidnightCRServoSystem(String name1, CRServo.Direction direction1,
                                 String name2, CRServo.Direction direction2, HardwareMap hardwareMap) {
        this( new MidnightCRServo(name1, direction1, hardwareMap),
                new MidnightCRServo(name2, direction2, hardwareMap) );
    }

    public MidnightCRServoSystem(String name1, CRServo.Direction direction1,
                                 String name2, CRServo.Direction direction2,
                                 String name3, CRServo.Direction direction3, HardwareMap hardwareMap) {
        this( new MidnightCRServo(name1, direction1, hardwareMap),
                new MidnightCRServo(name2, direction2, hardwareMap),
                new MidnightCRServo(name3, direction3, hardwareMap) );
    }

    public MidnightCRServoSystem(String name1, CRServo.Direction direction1,
                                 String name2, CRServo.Direction direction2,
                                 String name3, CRServo.Direction direction3,
                                 String name4, CRServo.Direction direction4, HardwareMap hardwareMap) {
        this(new MidnightCRServo(name1, direction1, hardwareMap),
                new MidnightCRServo(name2, direction2, hardwareMap),
                new MidnightCRServo(name3, direction3, hardwareMap),
                new MidnightCRServo(name4, direction4, hardwareMap));
    }

    public MidnightCRServoSystem(MidnightCRServo one, MidnightCRServo two) {
        servo1 = one; servo2 = two; servo3 = null; servo4 = null;
        servos = Arrays.asList(servo1, servo2);
    }

    public MidnightCRServoSystem(MidnightCRServo one, MidnightCRServo two, MidnightCRServo three) {
        servo1 = one; servo2 = two; servo3 = three; servo4 = null;
        servos = Arrays.asList(servo1, servo2, servo3);
    }

    public MidnightCRServoSystem(MidnightCRServo one, MidnightCRServo two, MidnightCRServo three, MidnightCRServo four) {
        servo1 = one; servo2 = two; servo3 = three; servo4 = four;
        servos = Arrays.asList(servo1, servo2, servo3, servo4);
    }


    public MidnightCRServoSystem setLimits(MidnightLimitSwitch min, MidnightLimitSwitch max) {
        for (MidnightCRServo s : servos) s.setLimits(min, max);
        return this;
    }
    public MidnightCRServoSystem setLimit(MidnightLimitSwitch min) {
        for (MidnightCRServo s: servos) s.setLimit(min);
        return this;
    }

    public void setPower(double power) {
        for (MidnightCRServo s : servos) s.setPower(power);
    }

    public double getPower() {return servo1.getPower();}

    @Override
    public String getName() {
        return "SYSTEM";
    }

    @Override
    public String[] getDash() {
        return new String[0];
    }
}
