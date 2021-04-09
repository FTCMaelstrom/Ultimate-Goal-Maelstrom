package org.firstinspires.ftc.teamcode.Mako.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mako.Robot.Mako;

import java.util.ArrayList;

import MidnightLibrary.MidnightResources.MidnightLinearOpMode;
import MidnightLibrary.MidnightRobot;

import static org.firstinspires.ftc.teamcode.Mako.Subsystems.Constants.INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.Mako.Subsystems.Constants.ROTATOR_POWER;
/**
 * Created by Amogh Mehta
 * Project: FtcRobotController_Ultimate-Goal_prod2
 * Last Modified: 4/9/21 2:51 PM
 * Last Updated: 4/9/21 2:55 PM
 **/
@TeleOp(name = "MakoTeleOp", group = "Mako")

public class MakoTeleOp extends MidnightLinearOpMode {
    /**
     * Define ROBOT
     **/
    private final Mako mako = new Mako();

    /**
     * Define VARIABLES
     **/
    boolean endgameModeEnabled = false;
    ArrayList<Boolean> booleanArrayList = new ArrayList<Boolean>();
    int booleanIncrement = 0;

    /**
     * Define ifPressed Toggle Method for Gamepad Buttons
     **/
    private boolean ifPressed(boolean button) {
        boolean output = false;
        if (booleanArrayList.size() == booleanIncrement) {
            booleanArrayList.add(false);
        }
        boolean buttonWas = booleanArrayList.get(booleanIncrement);
        if (button != buttonWas && button) {
            output = true;
        }
        booleanArrayList.set(booleanIncrement, button);
        booleanIncrement = booleanIncrement + 1;
        return output;
    }

    /**
     * Define ifPressedAnalog Toggle Method for Analog Gamepad Buttons (i.e. Triggers)
     **/
    private boolean ifPressedAnalog(double button) {
        boolean output = false;
        boolean buttonBoolean = false;
        if (button >= 0.1) {
            buttonBoolean = true;
        }
        if (booleanArrayList.size() == booleanIncrement) {
            booleanArrayList.add(false);
        }
        boolean buttonWas = booleanArrayList.get(booleanIncrement);
        if (buttonBoolean != buttonWas && buttonBoolean) {
            output = true;
        }
        booleanArrayList.set(booleanIncrement, buttonBoolean);
        booleanIncrement = booleanIncrement + 1;
        return output;
    }

    @Override
    public void runLinearOpMode() {
        mako.init(hardwareMap, MidnightRobot.OpMode.TELEOP);

        /**Run ONCE on init() **/
        while (!opModeIsActive()) {
            dash.create("Ready to go");
            dash.update();
        }

        waitForStart();

        /**Run REPEATEDLY in OpMode **/
        while (opModeIsActive()) {
            mako.MECH();

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


            boolean G1Pressed = ifPressed(gamepad1.a);
            if (G1Pressed && !endgameModeEnabled) {
                endgameModeEnabled = true;
            } else if (G1Pressed && endgameModeEnabled) {
                endgameModeEnabled = false;
            }

            dash.create("endgameModeEnabled?: ", endgameModeEnabled);
            dash.update();

            if (!endgameModeEnabled) {
                /**Trigger based variable speed intake control with adjustable speed control via dpad **/
                mako.intake.setPower(INTAKE_POWER * gamepad1.left_trigger - gamepad2.right_trigger);
                if (gamepad1.dpad_left) {
                    INTAKE_POWER -= 0.01;
                } else if (gamepad1.dpad_right) {
                    INTAKE_POWER += 0.01;
                }

                /*
                boolean G1RTPressed = ifPressedAnalog(gamepad1.right_trigger);
                boolean G1LTPressed = ifPressedAnalog(gamepad1.left_trigger);
                if (G1RTPressed & !G1LTPressed) {
                    mako.intake.setPower(1);
                } else if (!G1RTPressed & G1LTPressed) {
                    mako.intake.setPower(-1);
                }
                 */
            } else {
                mako.claw.driverControl(gamepad1);

                /**Trigger based variable speed rotator control with adjustable speed control via dpad **/
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
            booleanIncrement = 0;
        }
    }
}
