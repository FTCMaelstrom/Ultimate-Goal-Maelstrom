package org.firstinspires.ftc.teamcode.Mako.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mako.Robot.Mako;

import MidnightLibrary.MidnightResources.MidnightLinearOpMode;
import MidnightLibrary.MidnightRobot;

import static org.firstinspires.ftc.teamcode.Mako.Subsystems.Constants.ROTATOR_POWER;

/**
 * Created by Amogh Mehta
 * Project: FtcRobotController_Ultimate-Goal_prod2
 * Last Modified: 4/5/21 3:57 PM
 * Last Updated: 4/5/21 3:57 PM
 **/

@TeleOp(name = "Mako1", group = "Mako")
public class Mako1 extends MidnightLinearOpMode {
    private Mako mako = new Mako();
    boolean endgameModeEnabled = false;

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

            mako.rotator.setStopMotor();
            mako.rotator.resetEncoder();
            mako.rotator.runWithoutEncoderNoZeroPowerSet();
            mako.rotator.brakeMode();
            mako.rotator.setClosedLoop(false);

            /*
            // This is the button based rotator control
            if (gamepad1.right_bumper && !gamepad1.left_bumper) {
                mako.rotator.setPower(ROTATOR_POWER);
            }
            if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
                mako.rotator.setPower(0);
            }
            if (gamepad1.left_bumper && !gamepad1.right_bumper) {
                mako.rotator.setPower(-ROTATOR_POWER);
            }
            */

            // This is the claw servo control
            mako.claw.driverControl(gamepad1);

            packet.put("Rotator Speed: ", ROTATOR_POWER);
            packet.put("Rotator Motor Power: ", mako.rotator.getPower());
            packet.put("Rotator Velocity: ", mako.rotator.getVelocity());
            dashboard.sendTelemetryPacket(packet);

            if (gamepad1.a) {
                endgameModeEnabled = true;
            } else if (gamepad1.b) {
                endgameModeEnabled = false;
            }

            dash.create("endgameModeEnabled: ", endgameModeEnabled);

            while (endgameModeEnabled) {
                mako.rotator.setPower(ROTATOR_POWER * gamepad1.left_trigger - gamepad1.right_trigger);
                if (gamepad1.dpad_down) {
                    ROTATOR_POWER -= 0.01;
                } else if (gamepad1.dpad_up) {
                    ROTATOR_POWER = +0.01;
                }

                if (gamepad1.right_bumper) {
                    mako.claw.close();
                } else if (gamepad1.left_bumper) {
                    mako.claw.open();
                }
            }

        }
    }
}
