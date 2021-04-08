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
 * Last Modified: 4/8/21 3:17 PM
 * Last Updated: 4/8/21 3:22 PM
 **/
@TeleOp(name = "Mako1", group = "Mako")
public class Mako1 extends MidnightLinearOpMode {
    private final Mako mako = new Mako();

    @Override
    public void runLinearOpMode() {
        mako.init(hardwareMap, MidnightRobot.OpMode.TELEOP);

        while (!opModeIsActive()) {
            dash.create("Ready to go");
            dash.update();
        }

        waitForStart();

        while (opModeIsActive()) {
            mako.MECH();

            boolean endgameModeEnabled = false;

            FtcDashboard dashboard = FtcDashboard.getInstance();
            TelemetryPacket packet = new TelemetryPacket();

            mako.rotator.accessoryMotorConditioner();
            mako.rotator.setClosedLoop(false);

            packet.put("LeftFrontPower: ", mako.driveTrain.getPowerLeftFront());
            packet.put("LeftBackPower: ", mako.driveTrain.getPowerLeftBack());
            packet.put("RightFrontPower: ", mako.driveTrain.getPowerRightFront());
            packet.put("RightBackPower: ", mako.driveTrain.getPowerRightBack());

            packet.put("LeftFrontVelocity: ", mako.driveTrain.getVelocityLeftFront());
            packet.put("LeftBackVelocity: ", mako.driveTrain.getVelocityLeftBack());
            packet.put("RightFrontVelocity: ", mako.driveTrain.getVelocityRightFront());
            packet.put("RightBackVelocity: ", mako.driveTrain.getVelocityRightBack());
            dashboard.sendTelemetryPacket(packet);

            //Pressing a on gamepad activates endgameMode, pressing b on gamepad deactivates endgameMode
            if (gamepad1.a) {
                endgameModeEnabled = true;
            } else if (gamepad1.b) {
                endgameModeEnabled = false;
            }

            dash.create("endgameModeEnabled?: ", endgameModeEnabled);
            dash.update();

            if (!endgameModeEnabled) {
                dash.create("endgameModeDisabled");
                //There's nothing here because we don't have an intake, flywheel, or flicker
            } else if (endgameModeEnabled) {
                dash.create("endgameModeEnabled");
                mako.claw.driverControl(gamepad1);

                /*// This is the button based rotator control
                if (gamepad1.right_bumper && !gamepad1.left_bumper) {
                    mako.rotator.setPower(ROTATOR_POWER);
                }
                if (!gamepad1.left_bumper && !gamepad1.right_bumper) {
                    mako.rotator.setPower(0);
                }
                if (gamepad1.left_bumper && !gamepad1.right_bumper) {
                    mako.rotator.setPower(-ROTATOR_POWER);
                }*/

                //Trigger based rotator control with variable speed control via dpad
                mako.rotator.setPower(ROTATOR_POWER * gamepad1.left_trigger - gamepad1.right_trigger);
                if (gamepad1.dpad_down) {
                    ROTATOR_POWER -= 0.01;
                } else if (gamepad1.dpad_up) {
                    ROTATOR_POWER += 0.01;
                }

                if (gamepad1.right_bumper) {
                    mako.claw.close();
                } else if (gamepad1.left_bumper) {
                    mako.claw.open();
                }
            }
            packet.put("Rotator Speed: ", ROTATOR_POWER);
            packet.put("Rotator Motor Power: ", mako.rotator.getPower());
            packet.put("Rotator Velocity: ", mako.rotator.getVelocity());
            dashboard.sendTelemetryPacket(packet);

            dash.clear();
        }
    }
}
