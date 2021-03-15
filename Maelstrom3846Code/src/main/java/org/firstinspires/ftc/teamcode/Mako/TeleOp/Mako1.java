package org.firstinspires.ftc.teamcode.Mako.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mako.Robot.Mako;

import Library17822.MidnightWrappers.MidnightLinearOpMode;

@TeleOp(name = "Mako1", group = "Mako")
public class Mako1 extends MidnightLinearOpMode {
    private Mako mako = new Mako();

    @Override
    public void runLinearOpMode() {
        mako.init(hardwareMap);

        while (!opModeIsActive()) {
            dash.create("Ready to go");
            dash.update();
        }

        waitForStart();

        while(opModeIsActive()) {
            mako.MECH();
            dash.update();
        }
    }
}
