package org.firstinspires.ftc.teamcode.Mako.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Mako.Robot.Mako;

import MasqVision.RingDetector;
import MidnightLibrary.MidnightAuxiliary.MidnightLinearOpMode;

import static MidnightLibrary.MidnightRobot.OpMode.AUTO;

/*
 * Created by Amogh Mehta
 * Modified 5/9/21 10:00 PM by Amogh Mehta
 */
@Autonomous(name = "MakoAuto", group = "MakoAuto")
public class MakoAuto extends MidnightLinearOpMode {
    private final Mako mako = new Mako();

    @Override
    public void runLinearOpMode() {
        mako.init(hardwareMap, AUTO);
        RingDetector detector = (RingDetector) mako.cameraView.detector;

        while (!opModeIsActive()) {

            if (isStopRequested()) {
                mako.cameraView.stop();
                break;
            }
        }

        waitForStart();

        timeoutClock.reset();
        mako.tracker.reset();

        mako.driveTrain.setPower(0.4);
        sleep(3100);
        mako.driveTrain.setPower(0);
    }
}
