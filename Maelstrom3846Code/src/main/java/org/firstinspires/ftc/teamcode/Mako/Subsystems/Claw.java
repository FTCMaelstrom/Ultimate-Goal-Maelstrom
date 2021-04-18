package org.firstinspires.ftc.teamcode.Mako.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import MidnightLibrary.MidnightMotor.MidnightServo;
import MidnightLibrary.MidnightResources.MidnightHardware;
import MidnightLibrary.MidnightResources.MidnightSubSystem;
/**
 * Created by Amogh Mehta
 * Project: FtcRobotController_Ultimate-Goal_prod2
 * Last Modified: 4/8/21 3:17 PM
 * Last Updated: 4/8/21 3:22 PM
 **/
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
        claw.scaleRange(0.1, 0.4);
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
