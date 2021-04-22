package org.firstinspires.ftc.teamcode.Mako.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Mako.Robot.Mako;

import MasqVision.RingDetector;
import MidnightLibrary.MidnightAuxiliary.MidnightLinearOpMode;
import MidnightLibrary.MidnightMath.MidnightWayPoint;

import static MasqVision.RingDetector.TargetZone.B;
import static MasqVision.RingDetector.TargetZone.C;
import static MidnightLibrary.MidnightMath.MidnightWayPoint.PointMode.SWITCH;
import static MidnightLibrary.MidnightMath.MidnightWayPoint.PointMode.TANK;
import static MidnightLibrary.MidnightRobot.OpMode.AUTO;
/*
 * Created by Amogh Mehta
 * Modified 4/21/21 10:57 AM by Amogh Mehta
 */
@Autonomous(name = "Red", group = "MakoAuto")
public class MakoAuto1 extends MidnightLinearOpMode {
    int iterations;
    private final Mako mako = new Mako();
    private RingDetector.TargetZone zone;
    private final MidnightWayPoint target = new MidnightWayPoint().setTimeout(5).setSwitchMode(SWITCH).setTargetRadius(5).setAngularCorrectionSpeed(0.004).setPointSwitchRadius(24).setName("Drop Zone");
    private final MidnightWayPoint strafe = new MidnightWayPoint(-5, -30, 0).setSwitchMode(TANK).setAngularCorrectionSpeed(0.002);
    //stack = new MidnightWayPoint(4, 30, 0).setSwitchMode(TANK).setName("Starter Stack");

    @Override
    public void runLinearOpMode() {
        mako.init(hardwareMap, AUTO);
        RingDetector detector = (RingDetector) mako.cameraView.detector;

        while (!opModeIsActive()) {
            zone = detector.findZone();

            dash.create("Zone:", zone);
            dash.create("Control:", detector.getControl());
            dash.create("Top:", detector.getTop());
            dash.create("Bottom:", detector.getBottom());
            dash.update();

            if (isStopRequested()) {
                mako.cameraView.stop();
                break;
            }
        }

        waitForStart();

        //robot.driveTrain.setPower(0.5);
        timeoutClock.reset();
        mako.cameraView.stop();
        mako.tracker.reset();

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

        /* Here we are setting the zone target coordinates*/
        if (zone == B) {
            iterations = 1;
            target.setPoint(0, 1, 0);
        } else if (zone == C) {
            iterations = 3;
            target.setPoint(0, 11, 0);
        } else {
            target.setPoint(0, 21, 0);
        }

        //robot.xyPath();
        //robot.xyPath(new MidnightWayPoint(0, 0, 0).setTimeout(5).setDriveCorrectionSpeed(0.008).setAngularCorrectionSpeed(0.07));
        //MidnightWayPoint park = new MidnightWayPoint(0, 42, robot.tracker.getHeading()).setName("Park");
        //robot.xyPath(park);

        mako.driveTrain.setPower(0.4);
        sleep(3100);
        mako.driveTrain.setPower(0);
    }
}
