package org.firstinspires.ftc.teamcode.Mako.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

<<<<<<< Updated upstream
import Library17822.MidnightControlSystems.MidnightPID.MidnightPIDController;
import Library17822.MidnightDriveTrains.MidnightMechanumDriveTrain;
import Library17822.MidnightMotors.MidnightMotor;
import Library17822.MidnightMotors.MidnightMotorModel;
import Library17822.MidnightPositionTracker;
import Library17822.MidnightRobot;
import Library17822.MidnightWrappers.MidnightDashBoard;

import static Library17822.MinightResources.MidnightUtils.angleController;
import static Library17822.MinightResources.MidnightUtils.driveController;
import static Library17822.MinightResources.MidnightUtils.setTracker;
import static Library17822.MinightResources.MidnightUtils.turnController;

=======
import MaelstromCV.CameraView;
import MaelstromCV.UltimateGoalSpecific.RingHunter;
import MidnightLibrary.MidnightMath.MidnightPIDController;
import MidnightLibrary.MidnightMovement.MidnightMechanumDriveTrain;
import MidnightLibrary.MidnightMovement.MidnightMotor;
import MidnightLibrary.MidnightMovement.MidnightMotorModel;
import MidnightLibrary.MidnightMovement.MidnightPositionTracker;
import MidnightLibrary.MidnightRobot;
import MidnightLibrary.MidnightResources.MidnightDashBoard;

import static MidnightLibrary.MidnightResources.MidnightUtils.angleController;
import static MidnightLibrary.MidnightResources.MidnightUtils.driveController;
import static MidnightLibrary.MidnightResources.MidnightUtils.setTracker;
import static MidnightLibrary.MidnightResources.MidnightUtils.turnController;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
/**
 * Created by Amogh Mehta
 * Project: FtcRobotController_Ultimate-Goal_prod2
 * Last Modified: 3/17/21 7:13 PM
 * Last Updated: 3/17/21 10:47 PM
 **/
>>>>>>> Stashed changes
public class Mako extends MidnightRobot {
    public MidnightMotor encoder1, encoder2;

    public CameraView cameraView;
    private int rings;

    @Override
    public void mapHardware (HardwareMap hardwareDevices) {
        driveTrain = new MidnightMechanumDriveTrain(hardwareDevices, MidnightMotorModel.ORBITAL20);

        dash = MidnightDashBoard.getDash();
    }

    @Override
    public void init(HardwareMap hardwareMap) {
        mapHardware(hardwareMap);

        encoder1 = new MidnightMotor("xEncoder", MidnightMotorModel.REVTHROUGHBORE,hardwareMap);
        encoder2 = new MidnightMotor("yEncoder", MidnightMotorModel.REVTHROUGHBORE, hardwareMap);
        tracker = new MidnightPositionTracker(encoder1,encoder2,hardwareMap);

        tracker.setPosition(MidnightPositionTracker.DeadWheelPosition.BOTH_CENTER);
        tracker.setXRadius(9.00);
        tracker.setTrackWidth(18.00);
        tracker.reset();

        setTracker(tracker);

        driveController = new MidnightPIDController(0.00);
        angleController = new MidnightPIDController(0.00);
        turnController = new MidnightPIDController(0.00);

        driveTrain.setClosedLoop(true);
        driveTrain.resetEncoders();
        driveTrain.setKp(1e-8);
    }

    public void initCamera() {
        RingHunter ringFinder = new RingHunter();
        ringFinder.setClippingMargins(600,150,300,950);
        cameraView = new CameraView(ringFinder, CameraView.CamSelect.PHONECAMFRONT, hardwareMap);;
    }

    public int getRings() {
        return rings;
    }

}
