package org.firstinspires.ftc.teamcode.Mako.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Mako.Subsystems.Claw;

import MasqVision.RingDetector;
import MasqVision.MasqCamera;
import MidnightLibrary.MidnightMath.MidnightPIDController;
import MidnightLibrary.MidnightMotor.MidnightServo;
import MidnightLibrary.MidnightMovement.MidnightMechanumDriveTrain;
import MidnightLibrary.MidnightMotor.MidnightMotor;
import MidnightLibrary.MidnightMotor.MidnightMotorModel;
import MidnightLibrary.MidnightMovement.MidnightPositionTracker;
import MidnightLibrary.MidnightResources.MidnightDashBoard;
import MidnightLibrary.MidnightRobot;

import static MidnightLibrary.MidnightResources.MidnightUtils.angleController;
import static MidnightLibrary.MidnightResources.MidnightUtils.setTracker;
import static MidnightLibrary.MidnightResources.MidnightUtils.turnController;
import static MidnightLibrary.MidnightRobot.OpMode.AUTO;
import static org.openftc.easyopencv.OpenCvCameraRotation.UPRIGHT;

/**
 * Created by Amogh Mehta
 * Project: FtcRobotController_Ultimate-Goal_prod2
 * Last Modified: 4/5/21 12:23 PM
 * Last Updated: 4/5/21 12:31 PM
 **/

public class Mako extends MidnightRobot {
    public MidnightMotor encoder1, encoder2, rotator;
    public MasqCamera cameraView;
    public Claw claw;


    @Override
    public void mapHardware (HardwareMap hardwareMap) {
        driveTrain = new MidnightMechanumDriveTrain(hardwareMap, MidnightMotorModel.ORBITAL20);

        //intake = new MidnightMotor("intake", MidnightMotorModel.ORBITAL20, hardwareMap);

        rotator = new MidnightMotor("rotator", MidnightMotorModel.ORBITAL20, hardwareMap);

        claw = new Claw(hardwareMap);

        encoder1 = new MidnightMotor("xEncoder", MidnightMotorModel.REVTHROUGHBORE,hardwareMap);
        encoder2 = new MidnightMotor("yEncoder", MidnightMotorModel.REVTHROUGHBORE,hardwareMap);
        tracker = new MidnightPositionTracker(encoder1, encoder2, hardwareMap);

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

        //driveController = new MidnightPIDController(0.00);
        angleController = new MidnightPIDController(0.00);
        turnController = new MidnightPIDController(0.00);

        driveTrain.resetEncoders();

        if(opMode == AUTO) {
            driveTrain.setKp(1e-9);
            initCamera();
        }
        else driveTrain.setKp(1e-8);
    }

    public void initCamera() {
        /*
        OpenCV Supported Resolutions:
        1280x960, 1280x720, 960x720, 960x540, 864x480, 768x432, 720x480, 640x480, 320x240, 176x144
         */
        RingDetector ringFinder = new RingDetector();
        ringFinder.setClippingMargins(96,55,70,55);
        cameraView = new MasqCamera(ringFinder);
        cameraView.start(UPRIGHT);
    }


}
