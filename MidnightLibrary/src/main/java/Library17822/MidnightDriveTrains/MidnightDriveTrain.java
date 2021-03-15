package Library17822.MidnightDriveTrains;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

import Library17822.MidnightMotors.MidnightMotor;
import Library17822.MidnightMotors.MidnightMotorModel;
import Library17822.MidnightMotors.MidnightMotorSystem;
import Library17822.MidnightSensors.MidnightEncoder;
import Library17822.MinightResources.MidnightHelpers.MidnightHardware;
import Library17822.MinightResources.MidnightUtils;


public class MidnightDriveTrain implements MidnightHardware {
    public MidnightMotorSystem leftDrive, rightDrive;
    private MidnightMotorModel defaultModel = MidnightMotorModel.REVHDHEX20;
    public MidnightDriveTrain(String name1, String name2, String name3, String name4, HardwareMap hardwareMap) {
        leftDrive = new MidnightMotorSystem(name1, DcMotor.Direction.REVERSE, name2, DcMotor.Direction.REVERSE, "LEFTDRIVE", hardwareMap, defaultModel);
        rightDrive = new MidnightMotorSystem(name3, DcMotor.Direction.FORWARD, name4, DcMotor.Direction.FORWARD, "RIGHTDRIVE", hardwareMap, defaultModel);
    }
    public MidnightDriveTrain(String name1, String name2, String name3, String name4, HardwareMap hardwareMap, MidnightMotorModel midnightMotorModel) {
        leftDrive = new MidnightMotorSystem(name1, DcMotor.Direction.REVERSE, name2, DcMotor.Direction.REVERSE, "LEFTDRIVE", hardwareMap, midnightMotorModel);
        rightDrive = new MidnightMotorSystem(name3, DcMotor.Direction.FORWARD, name4, DcMotor.Direction.FORWARD, "RIGHTDRIVE", hardwareMap, midnightMotorModel);
    }
    public MidnightDriveTrain(HardwareMap hardwareMap){
        leftDrive = new MidnightMotorSystem("leftFront", DcMotor.Direction.FORWARD, "leftBack", DcMotor.Direction.FORWARD, "LEFTDRIVE", hardwareMap, defaultModel);
        rightDrive = new MidnightMotorSystem("rightFront", DcMotor.Direction.REVERSE, "rightBack", DcMotor.Direction.REVERSE, "RIGHTDRIVE", hardwareMap, defaultModel);
    }
    public MidnightDriveTrain(HardwareMap hardwareMap, MidnightMotorModel motorModel){
        //FOLLOW DIRECTIONS OF THIS
        leftDrive = new MidnightMotorSystem("leftFront", DcMotor.Direction.FORWARD, "leftBack", DcMotor.Direction.FORWARD, "LEFTDRIVE", hardwareMap, motorModel);
        rightDrive = new MidnightMotorSystem("rightFront", DcMotor.Direction.REVERSE, "rightBack", DcMotor.Direction.REVERSE, "RIGHTDRIVE", hardwareMap, motorModel);
    }
    public MidnightDriveTrain(MidnightMotorSystem left, MidnightMotorSystem right) {
        leftDrive = left;
        rightDrive = right;
    }

    public void resetEncoders () {
        leftDrive.resetEncoders();
        rightDrive.resetEncoders();
    }

    public void setVelocity(double leftPower, double rightPower) {
        rightDrive.setVelocity(rightPower);
        leftDrive.setVelocity(leftPower);
    }

    public void setVelocity(double leftFront, double leftBack, double rightFront, double rightBack) {
        leftDrive.motor1.setVelocity(leftFront);
        leftDrive.motor2.setVelocity(leftBack);
        rightDrive.motor1.setVelocity(rightFront);
        rightDrive.motor2.setVelocity(rightBack);
    }

    public void setVelocity(double power){
        leftDrive.setVelocity(power);
        rightDrive.setVelocity(power);
    }
    public double getInches() {
        return (leftDrive.getInches() + rightDrive.getInches())/2;
    }

    public void setPower(double power) {
        leftDrive.setPower(power);
        rightDrive.setPower(power);
    }
    public void setPower(double leftPower, double rightPower) {
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }


    public double getVelocity() {
        return (leftDrive.getVelocity() + rightDrive.getVelocity())/2;
    }
    public double getPower() {
        return (leftDrive.getPower() + rightDrive.getPower()) /2;
    }
    public void setVelocityLeft(double power) {
        leftDrive.setVelocity(power);
    }
    public void setVelocityRight(double power) {
        rightDrive.setVelocity(power);
    }
    public void runUsingEncoder() {
        leftDrive.runUsingEncoder();
        rightDrive.runUsingEncoder();
    }
    public void runWithoutEncoders() {
        leftDrive.runUsingEncoder();
        rightDrive.runUsingEncoder();
    }
    private boolean opModeIsActive() {
        return MidnightUtils.opModeIsActive();
    }
    public void zeroPowerBehavior(){
        rightDrive.breakMotors();
    }
    public double getCurrentPosition() {
        return (leftDrive.getCurrentPosition() + rightDrive.getCurrentPosition())/2;
    }
    public double getCurrentPositionPositive() {
        return (Math.abs(leftDrive.motor1.getCurrentPosition()) +
                Math.abs(leftDrive.motor2.getCurrentPosition()) +
                Math.abs(rightDrive.motor1.getCurrentPosition()) +
                Math.abs(rightDrive.motor1.getCurrentPosition()))/4;
    }
    public double getAbsolutePositon () {
        return (leftDrive.getAbsolutePosition() + rightDrive.getAbsolutePosition())/2;
    }

    public void setClosedLoop (boolean closedLoop) {
        leftDrive.setClosedLoop(closedLoop);
        rightDrive.setClosedLoop(closedLoop);
    }

    public void setKp(double kp){
        leftDrive.setKp(kp);
        rightDrive.setKp(kp);
    }
    public void setKi(double ki) {
        leftDrive.setKi(ki);
        rightDrive.setKi(ki);
    }
    public void setKd(double kd) {
        leftDrive.setKd(kd);
        rightDrive.setKd(kd);
    }

    public void setDefaultModel(MidnightMotorModel defaultModel) {this.defaultModel = defaultModel;}

    public MidnightEncoder getEncoder () {return rightDrive.motor1.getEncoder();}

    public List<MidnightEncoder> getEncoders() {
        List<MidnightEncoder> encoders = new ArrayList<>();
        encoders.add(leftDrive.motor1.getEncoder());
        encoders.add(leftDrive.motor2.getEncoder());
        encoders.add(rightDrive.motor1.getEncoder());
        encoders.add(rightDrive.motor2.getEncoder());
        return encoders;
    }

    public List<MidnightMotor> getMotors () {
        List<MidnightMotor> allMotors = new ArrayList<>(leftDrive.motors);
        allMotors.addAll(rightDrive.motors);
        return allMotors;
    }

    public String getName() {
        return "DRIVETRAIN";
    }
    public String[] getDash() {
        return new String[]{
                "Rate "+ getVelocity(),
                "Left Position: " + leftDrive.getAbsolutePosition(),
                "Right Position: " + rightDrive.getAbsolutePosition(),
        };
    }
}