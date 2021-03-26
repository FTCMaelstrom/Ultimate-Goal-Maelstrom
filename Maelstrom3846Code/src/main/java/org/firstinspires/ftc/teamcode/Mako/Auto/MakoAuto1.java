package org.firstinspires.ftc.teamcode.Mako.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Mako.Robot.Mako;

import MasqVision.RingDetector;
import MidnightLibrary.MidnightMath.MidnightWayPoint;
import MidnightLibrary.MidnightResources.MidnightLinearOpMode;

import static MidnightLibrary.MidnightMath.MidnightWayPoint.PointMode.SWITCH;
import static MidnightLibrary.MidnightMath.MidnightWayPoint.PointMode.TANK;
import static MidnightLibrary.MidnightRobot.OpMode.AUTO;

/**
 * Created by Amogh Mehta
 * Project: FtcRobotController_Ultimate-Goal_prod2
 * Last Modified: 3/26/21 1:24 AM
 * Last Updated: 3/26/21 1:24 AM
 **/
@Autonomous(name="Red", group = "MakoAuto")
public class MakoAuto1 extends MidnightLinearOpMode {
    private Mako robot = new Mako();
    private RingDetector.TargetZone zone;
    int iterations;
    private MidnightWayPoint target = new MidnightWayPoint().setTimeout(5).setSwitchMode(SWITCH).setTargetRadius(5).setAngularCorrectionSpeed(0.004).setPointSwitchRadius(24).setName("Drop Zone"),
            strafe = new MidnightWayPoint(-5,-30,0).setSwitchMode(TANK).setAngularCorrectionSpeed(0.002),
            stack = new MidnightWayPoint(4, 30, 0).setSwitchMode(TANK).setName("Starter Stack");

    @Override
    public void runLinearOpMode() {
        robot.init(hardwareMap, AUTO);
        RingDetector detector = (RingDetector) robot.cameraView.detector;

        while (!opModeIsActive()) {
            zone = detector.findZone();

            dash.create("Zone:", zone);
            dash.create("Control:", detector.getControl());
            dash.create("Top:", detector.getTop());
            dash.create("Bottom:", detector.getBottom());
            dash.update();

            if (isStopRequested()) {
                robot.cameraView.stop();
                break;
            }
        }

        waitForStart();

        timeoutClock.reset();
        robot.cameraView.stop();
        robot.tracker.reset();

        robot.xyPath(new MidnightWayPoint(7,-64, 180).setTimeout(5).setDriveCorrectionSpeed(0.008).setAngularCorrectionSpeed(0.07));

        MidnightWayPoint park = new MidnightWayPoint(robot.tracker.getGlobalX(), -72, robot.tracker.getHeading()).setDriveCorrectionSpeed(0.025).setAngularCorrectionSpeed(0.03).setName("Park");
        robot.xyPath(park);
    }
}
