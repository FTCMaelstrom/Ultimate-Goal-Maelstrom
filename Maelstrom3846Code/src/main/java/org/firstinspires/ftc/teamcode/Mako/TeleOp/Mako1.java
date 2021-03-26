package org.firstinspires.ftc.teamcode.Mako.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mako.Robot.Mako;

import MidnightLibrary.MidnightMovement.MidnightDriveTrain;
import MidnightLibrary.MidnightMovement.MidnightMechanumDriveTrain;
import MidnightLibrary.MidnightResources.MidnightLinearOpMode;
import MidnightLibrary.MidnightRobot;

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
        mako.init(hardwareMap, MidnightRobot.OpMode.TELEOP);

        while (!opModeIsActive()) {
            dash.create("Ready to go");

            dash.update();
        }

        waitForStart();

        while(opModeIsActive()) {
            mako.MECH();

            dash.create("leftFront: ", mako.driveTrain.getPowerLeftFront());
            dash.create("leftBack: ", mako.driveTrain.getPowerLeftBack());
            dash.create("rightFront: ", mako.driveTrain.getPowerRightFront());
            dash.create("rightBack: ", mako.driveTrain.getPowerRightBack());


            dash.update();
        }
    }
}
