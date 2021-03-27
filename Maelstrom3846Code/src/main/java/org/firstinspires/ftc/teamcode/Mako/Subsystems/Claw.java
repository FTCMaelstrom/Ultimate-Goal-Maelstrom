package org.firstinspires.ftc.teamcode.Mako.Subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import MidnightLibrary.MidnightMotor.MidnightServo;
import MidnightLibrary.MidnightResources.MidnightHardware;
import MidnightLibrary.MidnightResources.MidnightSubSystem;

/**
 * Created by Amogh Mehta
 * Project: FtcRobotController_Ultimate-Goal_prod2
 * Last Modified: 3/27/21 8:54 AM
 * Last Updated: 3/27/21 8:54 AM
 **/
public class Claw implements MidnightSubSystem {
    private MidnightServo claw;

    public void close() {
        claw.setPosition(0);
    }

    public void open() {
        claw.setPosition(1);
    }

    public void reset() {
        claw.scaleRange(0.1,0.4);
        close();
    }
    public Claw (HardwareMap hardwareMap) {
        claw = new MidnightServo("claw", hardwareMap);
        reset();
    }

    @Override
    public void driverControl (Gamepad controller) {
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
        return new MidnightHardware[] {claw};
    }
}
