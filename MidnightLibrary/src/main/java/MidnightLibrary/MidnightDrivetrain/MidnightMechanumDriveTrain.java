package MidnightLibrary.MidnightDrivetrain;

import com.qualcomm.robotcore.hardware.HardwareMap;

import MidnightLibrary.MidnightAuxiliary.MidnightHardware;
import MidnightLibrary.MidnightAuxiliary.MidnightUtils;
import MidnightLibrary.MidnightMovement.MidnightMotorModel;
import MidnightLibrary.MidnightMovement.MidnightMotorSystem;

import static MidnightLibrary.MidnightAuxiliary.MidnightUtils.BOOST_SPEED_MULTIPLIER;
import static MidnightLibrary.MidnightAuxiliary.MidnightUtils.DEFAULT_SPEED_MULTIPLIER;
import static MidnightLibrary.MidnightAuxiliary.MidnightUtils.adjustAngle;
import static MidnightLibrary.MidnightAuxiliary.MidnightUtils.angleController;
import static MidnightLibrary.MidnightAuxiliary.MidnightUtils.getTracker;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;

public class MidnightMechanumDriveTrain extends MidnightDriveTrain implements MidnightHardware {

    public MidnightMechanumDriveTrain(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    public MidnightMechanumDriveTrain(HardwareMap hardwareMap, MidnightMotorModel motorModel) {
        super(hardwareMap, motorModel);
    }

    public MidnightMechanumDriveTrain(MidnightMotorSystem left, MidnightMotorSystem right) {
        super(left, right);
    }

    public void setVelocityMECH(double angle, double speed, double targetHeading) {
        double turnPower = angleController.getOutput(adjustAngle(targetHeading - getTracker().getHeading()));
        angle = toRadians(angle);
        double adjustedAngle = angle + Math.PI / 4;
        double leftFront = (sin(adjustedAngle) * speed * BOOST_SPEED_MULTIPLIER) + turnPower * BOOST_SPEED_MULTIPLIER;
        double leftBack = (cos(adjustedAngle) * speed * DEFAULT_SPEED_MULTIPLIER) + turnPower * MidnightUtils.DEFAULT_TURN_MULTIPLIER;
        double rightFront = (cos(adjustedAngle) * speed * BOOST_SPEED_MULTIPLIER) - turnPower * BOOST_SPEED_MULTIPLIER;
        double rightBack = (sin(adjustedAngle) * speed * DEFAULT_SPEED_MULTIPLIER) - turnPower * MidnightUtils.DEFAULT_TURN_MULTIPLIER;
        double max = max(max(abs(leftFront), abs(leftBack)), max(abs(rightFront), abs(rightBack)));

        if (max > 1) {
            leftFront /= max;
            leftBack /= max;
            rightFront /= max;
            rightBack /= max;
        }
        leftDrive.motor1.setVelocity(leftFront);
        leftDrive.motor2.setVelocity(leftBack);
        rightDrive.motor1.setVelocity(rightFront);
        rightDrive.motor2.setVelocity(rightBack);
    }

    public void setVelocityMECH(double angle, double speed) {
        setVelocityMECH(angle, speed, getTracker().getHeading());
    }

    //getters designed to get the velocity control that is being exerted on the drivetrain system motors
    public double getVelocityLeftFront() {
        return (leftDrive.motor1.getVelocity());
    }

    public double getVelocityLeftBack() {
        return (leftDrive.motor2.getVelocity());
    }

    public double getVelocityRightFront() {
        return (rightDrive.motor1.getVelocity());
    }

    public double getVelocityRightBack() {
        return (rightDrive.motor2.getVelocity());
    }

    //getters designed to get the pure power values that the drivetrain system motors are giving
    public double getPowerLeftFront() {
        return (leftDrive.motor1.getPower());
    }

    public double getPowerLeftBack() {
        return (leftDrive.motor2.getPower());
    }

    public double getPowerRightFront() {
        return (rightDrive.motor1.getPower());
    }

    public double getPowerRightBack() {
        return (rightDrive.motor2.getPower());
    }
}
