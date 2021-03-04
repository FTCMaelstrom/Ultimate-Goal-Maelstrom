package Library17822.MidnightDriveTrains;

import com.qualcomm.robotcore.hardware.HardwareMap;

import Library17822.MidnightControlSystems.MidnightPID.MidnightPIDController;
import Library17822.MidnightMotors.MidnightMotorModel;
import Library17822.MidnightMotors.MidnightMotorSystem;
import Library17822.MidnightPositionTracker;
import Library17822.MinightResources.MidnightHelpers.MidnightHardware;
import Library17822.MinightResources.MidnightUtils;


public class MidnightMechanumDriveTrain extends MidnightDriveTrain implements MidnightHardware {
    public static MidnightPIDController angleCorrectionController = new MidnightPIDController(0.05);
    private MidnightPositionTracker tracker;

    public MidnightMechanumDriveTrain(HardwareMap hardwareMap){
        super(hardwareMap);
    }
    public MidnightMechanumDriveTrain(HardwareMap hardwareMap, MidnightMotorModel motorModel){
        super(hardwareMap, motorModel);
    }
    public MidnightMechanumDriveTrain(MidnightMotorSystem left, MidnightMotorSystem right) {
        super(left, right);
    }

    public void setVelocityMECH(double angle, double speed, double targetHeading) {
        double turnPower = angleCorrectionController.getOutput(MidnightUtils.adjustAngle(targetHeading - tracker.getHeading()));
        angle = Math.toRadians(angle);
        double adjustedAngle = angle + Math.PI/4;
        double leftFront = (Math.sin(adjustedAngle) * speed * MidnightUtils.DEFAULT_SPEED_MULTIPLIER) - turnPower * MidnightUtils.DEFAULT_TURN_MULTIPLIER;
        double leftBack = (Math.cos(adjustedAngle) * speed * MidnightUtils.DEFAULT_SPEED_MULTIPLIER) - turnPower  * MidnightUtils.DEFAULT_TURN_MULTIPLIER;
        double rightFront = (Math.cos(adjustedAngle) * speed * MidnightUtils.DEFAULT_SPEED_MULTIPLIER) + turnPower * MidnightUtils.DEFAULT_TURN_MULTIPLIER;
        double rightBack = (Math.sin(adjustedAngle) * speed * MidnightUtils.DEFAULT_SPEED_MULTIPLIER) + turnPower * MidnightUtils.DEFAULT_TURN_MULTIPLIER;
        double max = Math.max(Math.max(Math.abs(leftFront), Math.abs(leftBack)), Math.max(Math.abs(rightFront), Math.abs(rightBack)));
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

    public void setVelocityMECHXY(double angle, double speedx, double speedy, double targetHeading) {
        double turnPower = angleCorrectionController.getOutput(MidnightUtils.adjustAngle(targetHeading - tracker.getHeading()));
        angle = Math.toRadians(angle);
        double adjustedAngle = angle + Math.PI/4;

        double leftFrontX = (Math.sin(adjustedAngle) * speedx * MidnightUtils.DEFAULT_SPEED_MULTIPLIER);
        double leftBackX = (Math.cos(adjustedAngle) * speedx * MidnightUtils.DEFAULT_SPEED_MULTIPLIER);
        double rightFrontX = (Math.cos(adjustedAngle) * speedx * MidnightUtils.DEFAULT_SPEED_MULTIPLIER);
        double rightBackX = (Math.sin(adjustedAngle) * speedx * MidnightUtils.DEFAULT_SPEED_MULTIPLIER);

        double leftFrontY = (Math.sin(adjustedAngle) * speedy * MidnightUtils.DEFAULT_SPEED_MULTIPLIER);
        double leftBackY = (Math.cos(adjustedAngle) * speedy * MidnightUtils.DEFAULT_SPEED_MULTIPLIER);
        double rightFrontY = (Math.cos(adjustedAngle) * speedy * MidnightUtils.DEFAULT_SPEED_MULTIPLIER);
        double rightBackY = (Math.sin(adjustedAngle) * speedy * MidnightUtils.DEFAULT_SPEED_MULTIPLIER);


        double maxX = Math.max(Math.max(Math.abs(leftFrontX), Math.abs(leftBackX)), Math.max(Math.abs(rightFrontX), Math.abs(rightBackX)));
        double maxY = Math.max(Math.max(Math.abs(leftFrontY), Math.abs(leftBackY)), Math.max(Math.abs(rightFrontY), Math.abs(rightBackY)));

        if (maxX > 1) {
            leftFrontX /= maxX;
            leftBackX /= maxX;
            rightFrontX /= maxX;
            rightBackX /= maxX;
        }

        if (maxY > 1) {
            leftFrontY /= maxY;
            leftBackY /= maxY;
            rightFrontY /= maxY;
            rightBackY /= maxY;
        }

        double powerAdjustment = turnPower * MidnightUtils.DEFAULT_TURN_MULTIPLIER;
        leftDrive.motor1.setVelocity(leftFrontX + leftFrontY - powerAdjustment);
        leftDrive.motor2.setVelocity(leftBackX + leftBackY - powerAdjustment);
        rightDrive.motor1.setVelocity(rightFrontX + rightFrontY + powerAdjustment);
        rightDrive.motor2.setVelocity(rightBackX + rightBackY + powerAdjustment);
    }


    public void setVelocityMECH(double angle, double speed) {
        setVelocityMECH(angle, speed, tracker.getHeading());
    }
    public void setVelocityMECH(double angle, double speed, double targetHeading, double turnAdjustment) {
        angle = Math.toRadians(angle);
        double adjustedAngle = angle + Math.PI/4;
        double leftFront = (Math.sin(adjustedAngle) * speed * MidnightUtils.DEFAULT_SPEED_MULTIPLIER);
        double leftBack = (Math.cos(adjustedAngle) * speed * MidnightUtils.DEFAULT_SPEED_MULTIPLIER);
        double rightFront = (Math.cos(adjustedAngle) * speed * MidnightUtils.DEFAULT_SPEED_MULTIPLIER);
        double rightBack = (Math.sin(adjustedAngle) * speed * MidnightUtils.DEFAULT_SPEED_MULTIPLIER);
        leftFront -= turnAdjustment;
        leftBack -= turnAdjustment;
        rightBack += turnAdjustment;
        rightBack += turnAdjustment;
        double max = Math.max(Math.max(Math.abs(leftFront), Math.abs(leftBack)), Math.max(Math.abs(rightFront), Math.abs(rightBack)));
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

    public void setPowerMECH(double angle, double speed, double targetHeading, double turnAdjustment) {
        angle = Math.toRadians(angle);
        double adjustedAngle = angle + Math.PI/4;
        double leftFront = (Math.sin(adjustedAngle) * speed * MidnightUtils.DEFAULT_SPEED_MULTIPLIER);
        double leftBack = (Math.cos(adjustedAngle) * speed * MidnightUtils.DEFAULT_SPEED_MULTIPLIER);
        double rightFront = (Math.cos(adjustedAngle) * speed * MidnightUtils.DEFAULT_SPEED_MULTIPLIER);
        double rightBack = (Math.sin(adjustedAngle) * speed * MidnightUtils.DEFAULT_SPEED_MULTIPLIER);
        leftFront -= turnAdjustment;
        leftBack -= turnAdjustment;
        rightBack += turnAdjustment;
        rightBack += turnAdjustment;
        double max = Math.max(Math.max(Math.abs(leftFront), Math.abs(leftBack)), Math.max(Math.abs(rightFront), Math.abs(rightBack)));
        if (max > 1) {
            leftFront /= max;
            leftBack /= max;
            rightFront /= max;
            rightBack /= max;
        }
        leftDrive.motor1.setPower(leftFront);
        leftDrive.motor2.setPower(leftBack);
        rightDrive.motor1.setPower(rightFront);
        rightDrive.motor2.setPower(rightBack);
    }

    public void setTurnControllerKp (double kp) {
        angleCorrectionController.setKp(kp);
    }

    public void setPowerMECH(double angle, double speed, double targetHeading) {
        double turnPower = angleCorrectionController.getOutput(tracker.getHeading() - targetHeading);
        angle = Math.toRadians(angle);
        double adjustedAngle = angle + Math.PI/4;
        double leftFront = (Math.sin(adjustedAngle) * speed * MidnightUtils.DEFAULT_SPEED_MULTIPLIER) - turnPower * MidnightUtils.DEFAULT_TURN_MULTIPLIER;
        double leftBack = (Math.cos(adjustedAngle) * speed * MidnightUtils.DEFAULT_SPEED_MULTIPLIER) - turnPower  * MidnightUtils.DEFAULT_TURN_MULTIPLIER;
        double rightFront = (Math.cos(adjustedAngle) * speed * MidnightUtils.DEFAULT_SPEED_MULTIPLIER) + turnPower * MidnightUtils.DEFAULT_TURN_MULTIPLIER;
        double rightBack = (Math.sin(adjustedAngle) * speed * MidnightUtils.DEFAULT_SPEED_MULTIPLIER) + turnPower * MidnightUtils.DEFAULT_TURN_MULTIPLIER;
        double max = Math.max(Math.max(Math.abs(leftFront), Math.abs(leftBack)), Math.max(Math.abs(rightFront), Math.abs(rightBack)));
        if (max > 1) {
            leftFront /= max;
            leftBack /= max;
            rightFront /= max;
            rightBack /= max;
        }
        leftDrive.motor1.setPower(leftFront);
        leftDrive.motor2.setPower(leftBack);
        rightDrive.motor1.setPower(rightFront);
        rightDrive.motor2.setPower(rightBack);
    }

    public MidnightPositionTracker getTracker() {
        return tracker;
    }

    public void setTracker(MidnightPositionTracker tracker) {
        this.tracker = tracker;
    }
}
