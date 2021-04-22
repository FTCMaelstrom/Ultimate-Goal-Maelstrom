package org.firstinspires.ftc.teamcode.Mako.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import MidnightLibrary.MidnightAuxiliary.MidnightHardware;
import MidnightLibrary.MidnightAuxiliary.MidnightSubSystem;
import MidnightLibrary.MidnightMovement.MidnightServo;
/*
 * Created by Amogh Mehta
 * Modified 4/21/21 10:56 AM by Amogh Mehta
 */
public class Claw implements MidnightSubSystem {
    private final MidnightServo claw;

    public Claw(HardwareMap hardwareMap) {
        claw = new MidnightServo("claw", hardwareMap);
        reset();
    }

    public void close() {
        claw.setPosition(0);
    }

    public void open() {
        claw.setPosition(1);
    }

    public void reset() {
        claw.scaleRange(0.1, 0.8);
        close();
    }

    @Override
    public void driverControl(Gamepad controller) {
        claw.toggle(controller.a);
    }

    public MidnightServo getClaw() {
        return claw;
    }

    @Override
    public String getName() {
        return "Claw";
    }

    @Override
    public MidnightHardware[] getComponents() {
        return new MidnightHardware[]{claw};
    }
}
