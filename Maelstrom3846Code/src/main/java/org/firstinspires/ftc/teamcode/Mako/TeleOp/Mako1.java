package org.firstinspires.ftc.teamcode.Mako.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mako.Robot.Mako;

import MidnightLibrary.MidnightResources.MidnightLinearOpMode;
import MidnightLibrary.MidnightRobot;

/**
 * Created by Amogh Mehta
 * Project: FtcRobotController_Ultimate-Goal_prod2
 * Last Modified: 4/5/21 12:24 PM
 * Last Updated: 4/5/21 12:31 PM
 **/

@TeleOp(name = "Mako1", group = "Mako")
public class Mako1 extends MidnightLinearOpMode {
    private Mako mako = new Mako();
    double rotatorSpeed = 1;

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

            FtcDashboard dashboard = FtcDashboard.getInstance();
            TelemetryPacket packet = new TelemetryPacket();

            packet.put("LeftFrontPower: ", mako.driveTrain.getPowerLeftFront());
            packet.put("LeftBackPower: ", mako.driveTrain.getPowerLeftBack());
            packet.put("RightFrontPower: ", mako.driveTrain.getPowerRightFront());
            packet.put("RightBackPower: ", mako.driveTrain.getPowerRightBack());

            packet.put("LeftFrontVelocity: ", mako.driveTrain.getVelocityLeftFront());
            packet.put("LeftBackVelocity: ", mako.driveTrain.getVelocityLeftBack());
            packet.put("RightFrontVelocity: ", mako.driveTrain.getVelocityRightFront());
            packet.put("RightBackVelocity: ", mako.driveTrain.getVelocityRightBack());
            dashboard.sendTelemetryPacket(packet);

            /*
            dash.create("LFPow: ", mako.driveTrain.getPowerLeftFront());
            dash.create("LBPow: ", mako.driveTrain.getPowerLeftBack());
            dash.create("RFPow: ", mako.driveTrain.getPowerRightFront());
            dash.create("RBPow: ", mako.driveTrain.getPowerRightBack());
            */

            /*
            dash.create("LFVel: ", mako.driveTrain.getVelocityLeftFront());
            dash.create("LBVel: ", mako.driveTrain.getVelocityLeftBack());
            dash.create("RFVel: ", mako.driveTrain.getVelocityRightFront());
            dash.create("RBVel: ", mako.driveTrain.getVelocityRightBack());
            */


            mako.rotator.setStopMotor();
            mako.rotator.resetEncoder();
            mako.rotator.runWithoutEncoderNoZeroPowerSet();
            mako.rotator.brakeMode();
            mako.rotator.setClosedLoop(false);

            /* Below is a prototype rotator control */
            if (gamepad1.right_trigger > 0.05) {
                mako.rotator.setPower(gamepad1.right_trigger);
            }
            if (gamepad1.left_trigger > 0.05) {
                mako.rotator.setPower(-1*gamepad1.left_trigger);
            }

            /* This is the button based rotator control */
            if (gamepad1.right_bumper && !gamepad1.left_bumper) {
                mako.rotator.setPower(rotatorSpeed);
            }
            if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
                mako.rotator.setPower(0);
            }
            if (gamepad1.left_bumper && !gamepad1.right_bumper) {
                mako.rotator.setPower(-rotatorSpeed);
            }

            /* This is the claw servo control */
            mako.claw.driverControl(gamepad1);
            if (gamepad1.a) {
                mako.claw.open();
            }

            packet.put("Rotator Speed: ", rotatorSpeed);
            packet.put("Rotator Motor Power: ", mako.rotator.getPower());
            packet.put("Rotator Velocity: ", mako.rotator.getVelocity());

            /*
            dash.create("Rotator Speed: ", rotatorSpeed);
            dash.create("Rotator Motor: ", mako.rotator.getPower());
            dash.create("Rotator Velocity: ", mako.rotator.getVelocity());
            dash.update();
            */
        }
    }
}
