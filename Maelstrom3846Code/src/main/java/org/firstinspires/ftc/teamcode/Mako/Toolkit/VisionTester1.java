package org.firstinspires.ftc.teamcode.Mako.Toolkit;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mako.Robot.Mako;
import org.opencv.core.Rect;

import MasqVision.RingDetector;
import MidnightLibrary.MidnightAuxiliary.MidnightLinearOpMode;
import MidnightLibrary.MidnightMath.MidnightVector;

import static MidnightLibrary.MidnightAuxiliary.MidnightUtils.getCenterPoint;
import static MidnightLibrary.MidnightRobot.OpMode.AUTO;
/*
 * Modified 4/21/21 10:20 PM by Amogh Mehta
 */

@TeleOp(name = "VisionTester1", group = "Toolkit")
public class VisionTester1 extends MidnightLinearOpMode {
    private final Mako robot = new Mako();
    RingDetector detector;

    @Override
    public void runLinearOpMode() {
        robot.init(hardwareMap, AUTO);

        detector = (RingDetector) robot.cameraView.detector;

        while(!opModeIsActive()) {
            RingDetector.TargetZone zone = detector.findZone();

            dash.create("Zone:", zone);
            dash.create("Control:", detector.getControl());
            dash.create("Top:", detector.getTop());
            dash.create("Bottom:", detector.getBottom());
            dash.update();

            if(isStopRequested()) {
                robot.cameraView.stop();
                break;
            }
        }

        waitForStart();


        while (opModeIsActive()) {

            Rect rect = detector.getFoundRect();
            MidnightVector[] rings = detector.findRings();

            dash.create("Center Point X:", getCenterPoint(rect).x);
            dash.create("Height:", rect.height);


            if (rings[0] != null) {
                dash.create("Ring 1 X:", rings[0].getX());
                dash.create("Ring 1 Y:", rings[0].getY());
            }
            if (rings[1] != null) {
                dash.create("Ring 2 X:", rings[1].getX());
                dash.create("Ring 2 Y:", rings[1].getY());
            }

            dash.update();
        }
        robot.cameraView.stop();
    }
}