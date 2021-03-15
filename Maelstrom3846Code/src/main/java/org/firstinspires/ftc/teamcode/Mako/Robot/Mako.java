package org.firstinspires.ftc.teamcode.Mako.Robot;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Library17822.MidnightControlSystems.MidnightPID.MidnightPIDController;
import Library17822.MidnightDriveTrains.MidnightMechanumDriveTrain;
import Library17822.MidnightMotors.MidnightMotorModel;
import Library17822.MidnightPositionTracker;
import Library17822.MidnightRobot;
import Library17822.MidnightWrappers.MidnightDashBoard;

import static Library17822.MinightResources.MidnightUtils.angleController;
import static Library17822.MinightResources.MidnightUtils.driveController;
import static Library17822.MinightResources.MidnightUtils.setTracker;
import static Library17822.MinightResources.MidnightUtils.turnController;

public class Mako extends MidnightRobot {

    @Override
    public void mapHardware (HardwareMap hardwareDevices) {
        driveTrain = new MidnightMechanumDriveTrain(hardwareDevices, MidnightMotorModel.NEVEREST40);
        dash = MidnightDashBoard.getDash();
    }

    @Override
    public void init(HardwareMap hardwareMap) {
        mapHardware(hardwareMap);

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
