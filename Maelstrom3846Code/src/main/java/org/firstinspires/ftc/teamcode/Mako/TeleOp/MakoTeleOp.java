package org.firstinspires.ftc.teamcode.Mako.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mako.Robot.Mako;

import java.util.ArrayList;

import MidnightLibrary.MidnightResources.MidnightLinearOpMode;
import MidnightLibrary.MidnightRobot;

import static java.lang.Math.sqrt;
import static org.firstinspires.ftc.teamcode.Mako.Subsystems.Constants.INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.Mako.Subsystems.Constants.ROTATOR_POWER;
import static org.firstinspires.ftc.teamcode.Mako.Subsystems.Constants.VERSION_NUMBER;
/**
 * Created by Amogh Mehta
 * Project: FtcRobotController_Ultimate-Goal_prod2
 * Last Modified: 4/10/21 6:58 PM
 * Last Updated: 4/10/21 6:58 PM
 **/
@TeleOp(name = "MakoTeleOp", group = "Mako")

public class MakoTeleOp extends MidnightLinearOpMode {
    //INFO: Define ROBOT
    private final Mako mako = new Mako();

    //INFO: Define VARIABLES
    boolean endgameModeEnabled = false;
    ArrayList<Boolean> booleanArrayList = new ArrayList<>();
    int booleanIncrement = 0;
    double rotatorSpeed = ROTATOR_POWER;
    double intakeSpeed = INTAKE_POWER;

    //INFO: Define ifPressed State Machine Method for Gamepad Buttons
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

    //INFO: Define ifPressedAnalog State Machine Method for Analog Gamepad Buttons (i.e. Triggers)
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

        //INFO: Run ONCE on init()
        while (!opModeIsActive()) {
            mako.rotator.accessoryMotorConditioner();
            mako.rotator.setClosedLoop(false);

            dash.create("Setup completed, running version: ", VERSION_NUMBER);
            dash.update();
        }

        waitForStart();

        //INFO: Run REPEATEDLY in OpMode
        while (opModeIsActive()) {
            mako.MECH();

            FtcDashboard dashboard = FtcDashboard.getInstance();
            TelemetryPacket packet = new TelemetryPacket();

            /*
            packet.put("LeftFrontPower: ", mako.driveTrain.getPowerLeftFront());
            packet.put("LeftBackPower: ", mako.driveTrain.getPowerLeftBack());
            packet.put("RightFrontPower: ", mako.driveTrain.getPowerRightFront());
            packet.put("RightBackPower: ", mako.driveTrain.getPowerRightBack());

            packet.put("LeftFrontVelocity: ", mako.driveTrain.getVelocityLeftFront());
            packet.put("LeftBackVelocity: ", mako.driveTrain.getVelocityLeftBack());
            packet.put("RightFrontVelocity: ", mako.driveTrain.getVelocityRightFront());
            packet.put("RightBackVelocity: ", mako.driveTrain.getVelocityRightBack());
            */

            boolean G1APressed = ifPressed(gamepad1.a);
            if (G1APressed && !endgameModeEnabled) {
                endgameModeEnabled = true;
            } else if (G1APressed) {
                endgameModeEnabled = false;
            }

            dash.create("endgameModeEnabled?: ", endgameModeEnabled);
            dash.update();

            if (!endgameModeEnabled) {
                //INFO: Trigger based variable speed intake control with adjustable speed control via dpad
                mako.intake.setPower(intakeSpeed * gamepad1.left_trigger - gamepad1.right_trigger);
                boolean dpadUpPressed = ifPressed(gamepad1.dpad_up);
                boolean dpadDownPressed = ifPressed(gamepad1.dpad_down);
                if (dpadUpPressed && !dpadDownPressed) {
                    intakeSpeed += 0.01;
                } else if (dpadDownPressed & !dpadUpPressed) {
                    intakeSpeed -= 0.01;
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

                //INFO: Trigger based variable speed rotator control with adjustable speed control via dpad
                mako.rotator.setPower(rotatorSpeed * (gamepad1.left_trigger - gamepad1.right_trigger));
                boolean dpadDownPressed = ifPressed(gamepad1.dpad_down);
                boolean dpadUpPressed = ifPressed(gamepad1.dpad_up);
                if (dpadDownPressed && !dpadUpPressed) {
                    rotatorSpeed -= 0.01;
                } else if (dpadUpPressed && !dpadDownPressed) {
                    rotatorSpeed += 0.01;
                }

                //TODO: Test BOOST mode for rotator
                boolean G1XPressed = ifPressed(gamepad1.x);
                if (G1XPressed) {
                    rotatorSpeed = sqrt(2);
                }

                if (gamepad1.b) {
                    mako.claw.open();
                }

                /*
                boolean rightBumperPressed = ifPressed(gamepad1.right_bumper);
                boolean leftBumperPressed = ifPressed(gamepad1.left_bumper);
                if (rightBumperPressed) {
                    mako.claw.close();
                } else if (leftBumperPressed) {
                    mako.claw.open();
                }

                 */
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
