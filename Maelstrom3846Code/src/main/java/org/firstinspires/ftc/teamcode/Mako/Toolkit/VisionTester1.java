package org.firstinspires.ftc.teamcode.Mako.Toolkit;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mako.Robot.Mako;
import org.opencv.core.Rect;

import MasqVision.RingDetector;
import MidnightLibrary.MidnightMath.MidnightVector;
import MidnightLibrary.MidnightResources.MidnightLinearOpMode;
import MidnightLibrary.MidnightResources.MidnightUtils;

import static MidnightLibrary.MidnightRobot.OpMode.AUTO;

@TeleOp(name = "VisionTester1", group = "Toolkit")
public class VisionTester1 extends MidnightLinearOpMode {
    private final Mako mako = new Mako();
    RingDetector detector;

    @Override
    public void runLinearOpMode() {
        mako.init(hardwareMap, AUTO);
        detector = (RingDetector) mako.cameraView.detector;

        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        while (!opModeIsActive()) {

            RingDetector.TargetZone zone = detector.findZone();


            packet.put("Zone: ", zone);
            packet.put("Control: ", detector.getControl());
            packet.put("Top: ", detector.getTop());
            packet.put("Bottom: ", detector.getBottom());
            dashboard.sendTelemetryPacket(packet);

            if (isStopRequested()) {
                mako.cameraView.stop();
                break;
            }
        }

        waitForStart();
        detector.switchDetection();

        while (opModeIsActive()) {
            Rect rect = detector.getFoundRect();
            MidnightVector[] rings = detector.findRings();

            packet.put("Center Point X: ", MidnightUtils.getCenterPoint(rect).x);
            packet.put("Height: ", rect.height);

            if (rings[0] != null) {
                packet.put("Ring 1 X: ", rings[0].getX());
                packet.put("Ring 1 Y: ", rings[0].getY());
            }

            if (rings[0] != null) {
                packet.put("Ring 2 X: ", rings[0].getX());
                packet.put("Ring 2 Y: ", rings[0].getY());
            }
            dashboard.sendTelemetryPacket(packet);
        }
        mako.cameraView.stop();
    }
}