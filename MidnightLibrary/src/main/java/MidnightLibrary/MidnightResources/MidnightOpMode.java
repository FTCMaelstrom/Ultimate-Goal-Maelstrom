package MidnightLibrary.MidnightResources;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import MidnightLibrary.MidnightRobot;


/**
 * This is a Masquerade OpMode. It includes a MidnightRobot and Dashboard along with the custom gamepads.
 */

public abstract class MidnightOpMode extends OpMode {
    public MidnightRobot robot;
    public MidnightDashBoard dash = new MidnightDashBoard(super.telemetry);
    protected MidnightController controller1 = new MidnightController(super.gamepad1, "controller1");
    protected MidnightController controller2 = new MidnightController(super.gamepad2, "controller2");
}
