package MidnightLibrary.MidnightMovement;

import com.qualcomm.robotcore.hardware.HardwareMap;

import MidnightLibrary.MidnightMotor.MidnightMotorModel;
import MidnightLibrary.MidnightMotor.MidnightMotorSystem;
import MidnightLibrary.MidnightSensors.MidnightEncoder;


import static MidnightLibrary.MidnightMotor.MidnightMotorModel.ORBITAL20;
import static MidnightLibrary.MidnightMotor.MidnightMotorModel.REVHDHEX20;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.*;
import static java.lang.Math.abs;

public class MidnightDriveTrain {
    public MidnightMotorSystem leftDrive, rightDrive;

    public MidnightDriveTrain(String name1, String name2, String name3, String name4, HardwareMap hardwareMap) {
        leftDrive = new MidnightMotorSystem(name1, REVERSE, name2, REVERSE, "LEFTDRIVE", hardwareMap, ORBITAL20);
        rightDrive = new MidnightMotorSystem(name3, FORWARD, name4, FORWARD, "RIGHTDRIVE", hardwareMap, ORBITAL20);
    }
    public MidnightDriveTrain(String name1, String name2, String name3, String name4, HardwareMap hardwareMap, MidnightMotorModel MidnightMotorModel) {
        leftDrive = new MidnightMotorSystem(name1, REVERSE, name2, REVERSE, "LEFTDRIVE", hardwareMap, MidnightMotorModel);
        rightDrive = new MidnightMotorSystem(name3, FORWARD, name4, FORWARD, "RIGHTDRIVE", hardwareMap, MidnightMotorModel);
    }
    public MidnightDriveTrain(HardwareMap hardwareMap){
        leftDrive = new MidnightMotorSystem("leftFront", FORWARD, "leftBack", FORWARD, "LEFTDRIVE", hardwareMap, ORBITAL20);
        rightDrive = new MidnightMotorSystem("rightFront", REVERSE, "rightBack", REVERSE, "RIGHTDRIVE", hardwareMap, ORBITAL20);
    }
    public MidnightDriveTrain(HardwareMap hardwareMap, MidnightMotorModel motorModel){
        leftDrive = new MidnightMotorSystem("leftFront", FORWARD, "leftBack", FORWARD, "LEFTDRIVE", hardwareMap, motorModel);
        rightDrive = new MidnightMotorSystem("rightFront", REVERSE, "rightBack", REVERSE, "RIGHTDRIVE", hardwareMap, motorModel);
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
    public void setVelocity(double power){
        leftDrive.setVelocity(power);
        rightDrive.setVelocity(power);
    }
    public void setVelocity(double leftFront, double leftBack, double rightFront, double rightBack) {
        leftDrive.motor1.setVelocity(leftFront);
        leftDrive.motor2.setVelocity(leftBack);
        rightDrive.motor1.setVelocity(rightFront);
        rightDrive.motor2.setVelocity(rightBack);
    }

    public double getInches() {return (leftDrive.getInches() + rightDrive.getInches())/2;}

    public double getVelocity() {
        return (leftDrive.getVelocity() + rightDrive.getVelocity())/2;
    }
    public double getPower() {
        return (leftDrive.getPower() + rightDrive.getPower()) /2;
    }

    public int getCurrentPosition() {
        return (leftDrive.getCurrentPosition() + rightDrive.getCurrentPosition())/2;
    }
    public double getCurrentPositionPositive() {
        return (abs(leftDrive.motor1.getCurrentPosition()) +
                abs(leftDrive.motor2.getCurrentPosition()) +
                abs(rightDrive.motor1.getCurrentPosition()) +
                abs(rightDrive.motor1.getCurrentPosition()))/4;
    }

    public void setClosedLoop (boolean closedLoop) {
        leftDrive.setClosedLoop(closedLoop);
        rightDrive.setClosedLoop(closedLoop);
    }

    public void setKp(double kp){
        leftDrive.setKp(kp);
        rightDrive.setKp(kp);
    }

    public void startVelocityControl() {
        leftDrive.startVelocityControl();
        rightDrive.startVelocityControl();
    }

    public void runUsingEncoder() {
        leftDrive.runUsingEncoder();
        rightDrive.runUsingEncoder();
    }
    public void runWithoutEncoder() {
        leftDrive.runWithoutEncoder();
        rightDrive.runWithoutEncoder();
    }

    public MidnightEncoder getEncoder () {return rightDrive.motor1.getEncoder();}

    public String getName() {return "DRIVETRAIN";}
    public String[] getDash() {
        return new String[]{
                "Rate "+ getVelocity(),
                "Left Position: " + leftDrive.getAbsolutePosition(),
                "Right Position: " + rightDrive.getAbsolutePosition(),
        };
    }
}