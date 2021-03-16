package org.firstinspires.ftc.teamcode.Mako.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library17822.MidnightControlSystems.MidnightPID.MidnightPIDController;
import Library17822.MidnightDriveTrains.MidnightMechanumDriveTrain;
import Library17822.MidnightMotors.MidnightMotor;
import Library17822.MidnightMotors.MidnightMotorModel;
import Library17822.MidnightResources.MidnightPositionTracker;
import Library17822.MidnightRobot;
import Library17822.MidnightWrappers.MidnightDashBoard;

import static Library17822.MidnightResources.MidnightUtils.angleController;
import static Library17822.MidnightResources.MidnightUtils.driveController;
import static Library17822.MidnightResources.MidnightUtils.setTracker;
import static Library17822.MidnightResources.MidnightUtils.turnController;
/**
 * Created by Amogh Mehta
 * Project: FtcRobotController_Ultimate-Goal_prod2
 * Last Modified: 3/16/21 3:19 PM
 * Last Updated: 3/16/21 3:29 PM
 **/
public class Mako extends MidnightRobot {
    public MidnightMotor encoder1, encoder2;

    @Override
    public void mapHardware (HardwareMap hardwareDevices) {
        driveTrain = new MidnightMechanumDriveTrain(hardwareDevices, MidnightMotorModel.NEVEREST40);


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
    }

}
