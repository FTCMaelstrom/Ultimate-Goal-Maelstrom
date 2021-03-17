package org.firstinspires.ftc.teamcode.Mako.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mako.Robot.Mako;

import Library17822.MidnightResources.MidnightLinearOpMode;
/**
 * Created by Amogh Mehta
 * Project: FtcRobotController_Ultimate-Goal_prod2
 * Last Modified: 3/16/21 3:19 PM
 * Last Updated: 3/16/21 3:29 PM
 **/
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
