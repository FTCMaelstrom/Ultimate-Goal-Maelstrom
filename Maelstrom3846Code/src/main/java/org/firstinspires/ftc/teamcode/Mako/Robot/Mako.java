package org.firstinspires.ftc.teamcode.Mako.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import MasqVision.MasqCamera;
import MasqVision.RingDetector;
import MidnightLibrary.MidnightMath.MidnightPIDController;
import MidnightLibrary.MidnightMotor.MidnightMotorModel;
import MidnightLibrary.MidnightMovement.MidnightMechanumDriveTrain;
import MidnightLibrary.MidnightMovement.MidnightPositionTracker;
import MidnightLibrary.MidnightResources.MidnightDashBoard;
import MidnightLibrary.MidnightResources.MidnightUtils;
import MidnightLibrary.MidnightRobot;

import static MidnightLibrary.MidnightResources.MidnightUtils.setTracker;
import static MidnightLibrary.MidnightRobot.OpMode.AUTO;
import static org.openftc.easyopencv.OpenCvCameraRotation.UPRIGHT;

/*
 * Modified 4/20/21 9:07 PM by Amogh Mehta
 */

public class Mako extends MidnightRobot {
    //public MidnightMotor encoder1, encoder2, rotator, intake;
    public MasqCamera cameraView;
    //public Claw claw;


    @Override
    public void mapHardware(HardwareMap hardwareMap) {
        driveTrain = new MidnightMechanumDriveTrain(hardwareMap, MidnightMotorModel.ORBITAL20);

        //intake = new MidnightMotor("intake", MidnightMotorModel.ORBITAL20, hardwareMap);

        //rotator = new MidnightMotor("rotator", MidnightMotorModel.ORBITAL20, hardwareMap);

        //claw = new Claw(hardwareMap);

        //encoder1 = new MidnightMotor("xEncoder", MidnightMotorModel.REVTHROUGHBORE, hardwareMap);
        //encoder2 = new MidnightMotor("yEncoder", MidnightMotorModel.REVTHROUGHBORE, hardwareMap);
        tracker = new MidnightPositionTracker(hardwareMap);

        dash = MidnightDashBoard.getDash();

    }

    @Override
    public void init(HardwareMap hardwareMap, OpMode opMode) {
        mapHardware(hardwareMap);

        tracker.setPosition(MidnightPositionTracker.DeadWheelPosition.BOTH_CENTER);
        tracker.setXRadius(8.50);
        tracker.setTrackWidth(17.00);
        tracker.reset();

        setTracker(tracker);

        MidnightUtils.driveController = new MidnightPIDController(0.00);
        MidnightUtils.angleController = new MidnightPIDController(0.00);
        MidnightUtils.turnController = new MidnightPIDController(0.00);

        driveTrain.resetEncoders();

        if (opMode == AUTO) {
            driveTrain.setKp(1e-9);
            //driveTrain.setKi(0);
            //driveTrain.setKd(0);
            initCamera();
        } else {
            driveTrain.setKp(1e-8);
            //driveTrain.setKi(0);
            //driveTrain.setKd(0);
        }
    }

    public void initCamera() {
        /*
        OpenCV Supported Resolutions:
        1280x960, 1280x720, 960x720, 960x540, 864x480, 768x432, 720x480, 640x480, 320x240, 176x144
         */
        RingDetector ringFinder = new RingDetector();
        ringFinder.setClippingMargins(100, 100, 100, 100);
        cameraView = new MasqCamera(ringFinder);
        cameraView.start(UPRIGHT);
    }
}
