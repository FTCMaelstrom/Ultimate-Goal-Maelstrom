package org.firstinspires.ftc.teamcode.Mako.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;

import MaelstromCV.CameraView;
import MaelstromCV.UltimateGoalSpecific.RingDetector;
import MaelstromCV.UltimateGoalSpecific.RingHunter;
import MasqVision.MasqCVDetector;
import MasqVision.MasqCamera;
import MidnightLibrary.MidnightMath.MidnightPIDController;
import MidnightLibrary.MidnightMovement.MidnightDriveTrain;
import MidnightLibrary.MidnightMovement.MidnightMechanumDriveTrain;
import MidnightLibrary.MidnightMovement.MidnightMotor;
import MidnightLibrary.MidnightMovement.MidnightMotorModel;
import MidnightLibrary.MidnightMovement.MidnightPositionTracker;
import MidnightLibrary.MidnightResources.MidnightDashBoard;
import MidnightLibrary.MidnightRobot;

import static MidnightLibrary.MidnightResources.MidnightUtils.angleController;
import static MidnightLibrary.MidnightResources.MidnightUtils.driveController;
import static MidnightLibrary.MidnightResources.MidnightUtils.getHardwareMap;
import static MidnightLibrary.MidnightResources.MidnightUtils.setTracker;
import static MidnightLibrary.MidnightResources.MidnightUtils.turnController;
import static MidnightLibrary.MidnightRobot.OpMode.AUTO;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.openftc.easyopencv.OpenCvCameraRotation.SIDEWAYS_LEFT;
import static org.openftc.easyopencv.OpenCvCameraRotation.UPRIGHT;

/**
 * Created by Amogh Mehta
 * Project: FtcRobotController_Ultimate-Goal_prod2
 * Last Modified: 3/17/21 11:00 PM
 * Last Updated: 3/17/21 11:01 PM
 **/
public class Mako extends MidnightRobot {
    public MidnightMotor encoder1, encoder2;
    public MasqCamera cameraView;


    @Override
    public void mapHardware (HardwareMap hardwareMap) {
        driveTrain = new MidnightMechanumDriveTrain(hardwareMap, MidnightMotorModel.ORBITAL20);



        encoder1 = new MidnightMotor("xEncoder", MidnightMotorModel.REVTHROUGHBORE,hardwareMap);
        encoder2 = new MidnightMotor("yEncoder", MidnightMotorModel.REVTHROUGHBORE,hardwareMap);
        tracker = new MidnightPositionTracker(encoder1,encoder2,hardwareMap);

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
            driveTrain.setKp(5e-8);
            initCamera();
        }
        else driveTrain.setKp(5e-9);

        driveTrain.setKp(1e-8);//Set to 1e-3 by default, which is too low for a drivetrain, keep at 1e-8 or shaking will occur
    }

    public void initCamera() {
        RingDetector ringFinder = new RingDetector();
        ringFinder.setClippingMargins(200,225,80,215);
        cameraView = new MasqCamera(ringFinder);
        cameraView.start(UPRIGHT);
    }

    public int getRings() {return 0;}

}
