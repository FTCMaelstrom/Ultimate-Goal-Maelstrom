package Library17822.MidnightMotors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import Library17822.MidnightSensors.MidnightLimitSwitch;
import Library17822.MidnightResources.MidnightHelpers.MidnightHardware;

/**
 * MidnightMotorSystem That supports two or more motors and treats them as one
 */
public class MidnightMotorSystem implements MidnightHardware {
    public MidnightMotor motor1 , motor2, motor3;
    public List<MidnightMotor> motors;
    public int numMotors;
    double kp, ki, kd;
    private double currentPower = 0;
    private double slowDown = 0;
    private String systemName;
    private MidnightMotorModel encoder = MidnightMotorModel.ORBITAL20;
    public MidnightMotorSystem(String name1, DcMotor.Direction direction, String name2, DcMotor.Direction direction2, String systemName, HardwareMap hardwareMap, MidnightMotorModel encoder) {
        this.systemName = systemName;
        motor1 = new MidnightMotor(name1, encoder, direction, hardwareMap);
        motor2 = new MidnightMotor(name2, encoder, direction2, hardwareMap);
        motor3 = null;
        motors = Arrays.asList(motor1, motor2);
        numMotors = 2;//
    }
    public MidnightMotorSystem(String name1, String name2, String systemName, HardwareMap hardwareMap, MidnightMotorModel encoder) {
        this.systemName = systemName;
        motor1 = new MidnightMotor(name1, encoder, DcMotor.Direction.FORWARD, hardwareMap);
        motor2 = new MidnightMotor(name2, encoder, DcMotor.Direction.FORWARD, hardwareMap);
        motor3 = null;
        motors = Arrays.asList(motor1, motor2);
        numMotors = 2;
    }
    public MidnightMotorSystem(String name1, DcMotor.Direction direction,
                               String name2, DcMotor.Direction direction2,
                               String name3, DcMotor.Direction direction3, String systemName,
                               HardwareMap hardwareMap, MidnightMotorModel encoder) {
        this.systemName = systemName;
        motor1 = new MidnightMotor(name1, encoder, direction, hardwareMap);
        motor2 = new MidnightMotor(name2, encoder, direction2, hardwareMap);
        motor3 = new MidnightMotor(name3, encoder, direction3, hardwareMap);
        motors = Arrays.asList(motor1, motor2, motor3);
        numMotors = 3;
    }
    public MidnightMotorSystem(String name1, String name2, String name3, String systemName, HardwareMap hardwareMap, MidnightMotorModel encoder) {
        this.systemName = systemName;
        motor1 = new MidnightMotor(name1, encoder, DcMotor.Direction.FORWARD, hardwareMap);
        motor2 = new MidnightMotor(name2, encoder, DcMotor.Direction.FORWARD, hardwareMap);
        motor3 = new MidnightMotor(name3, encoder, DcMotor.Direction.FORWARD, hardwareMap);
        motors = Arrays.asList(motor1, motor2, motor3);
        numMotors = 3;
    }
    public MidnightMotorSystem(String name1, String name2, MidnightMotorModel encoder, HardwareMap hardwareMap) {
        motor1 = new MidnightMotor(name1, encoder, DcMotor.Direction.FORWARD, hardwareMap);
        motor2 = new MidnightMotor(name2, encoder, DcMotor.Direction.FORWARD, hardwareMap);
        motors = Arrays.asList(motor1, motor2);
        numMotors = 2;
    }
    public MidnightMotorSystem(String name1, DcMotor.Direction d1, String name2, DcMotor.Direction d2, MidnightMotorModel encoder, HardwareMap hardwareMap) {
        motor1 = new MidnightMotor(name1, encoder, d1, hardwareMap);
        motor2 = new MidnightMotor(name2, encoder, d2, hardwareMap);
        motors = Arrays.asList(motor1, motor2);
        numMotors = 2;
    }
    public MidnightMotorSystem(String name, MidnightMotorModel midnightMotorModel, DcMotor.Direction direction, HardwareMap hardwareMap) {
        motor1 = new MidnightMotor(name, midnightMotorModel, direction, hardwareMap);
        motors = Collections.singletonList(motor1);
        numMotors = 1;
    }
    public void setBreakMode() {
        for (MidnightMotor masqMotor : motors)
            masqMotor.setBreakMode();
    }
    public MidnightMotorSystem resetEncoders() {
        for (MidnightMotor masqMotor : motors)
            masqMotor.resetEncoder();
        return this;
    }
    public MidnightMotorSystem setKp(double kp){
        this.kp = kp;
        for (MidnightMotor masqMotor: motors) masqMotor.setKp(kp);
        return this;
    }
    public MidnightMotorSystem setKi(double ki){
        this.ki = ki;
        for (MidnightMotor masqMotor: motors) masqMotor.setKi(ki);
        return this;
    }
    public MidnightMotorSystem setKd(double kd){
        this.kd = kd;
        for (MidnightMotor masqMotor: motors) masqMotor.setKd(kd);
        return this;
    }
    public double getPower() {
        double num = 0, sum = 0;
        for (MidnightMotor masqMotor: motors) {
            sum += Math.abs(masqMotor.getPower());
            num++;
        }
        return sum/num;
    }
    public double getInches () {
        double num = 0, sum = 0;
        for (MidnightMotor masqMotor : motors) {
            sum += masqMotor.getEncoder().getInches();
            num++;
        }
        return sum/num;
    }
    public void setMinPower(double power) {
        for (MidnightMotor masqMotor : motors) masqMotor.setMinPower(power);
    }
    public void setVelocity(double power) {
        currentPower = power;
        for (MidnightMotor masqMotor : motors) masqMotor.setVelocity(power);
    }
    public void setPower(double power) {
        for (MidnightMotor masqMotor : motors)masqMotor.setPower(power);
    }
    public void setClosedLoop (boolean closedLoop) {
        for (MidnightMotor masqMotor : motors) {
            masqMotor.setClosedLoop(closedLoop);
        }
    }
    public void setLimits (MidnightLimitSwitch min, MidnightLimitSwitch max) {
        for (MidnightMotor masqMotor : motors) {
            masqMotor.setLimits(min, max);
        }
    }

    public MidnightMotorSystem setDistance(int distance){
        for (MidnightMotor masqMotor: motors)
            masqMotor.setDistance(distance);
        return this;
    }
    public MidnightMotorSystem runUsingEncoder() {
        for (MidnightMotor masqMotor: motors)
            masqMotor.runUsingEncoder();
        return this;
    }
    public MidnightMotorSystem breakMotors(){
        for (MidnightMotor masqMotor: motors)
            masqMotor.setBreakMode();
        return this;
    }
    public MidnightMotorSystem unBreakMotors(){
        for (MidnightMotor masqMotor: motors)
            masqMotor.unBreakMode();
        return this;
    }
    public MidnightMotorSystem runWithoutEncoders() {
        for (MidnightMotor masqMotor: motors)
            masqMotor.runWithoutEncoders();
        return this;
    }
    public double getAngle () {
        double sum = 0, num = 0;
        for (MidnightMotor masqMotor: motors) {
            sum += masqMotor.getAngle();
            num++;
        }
        return sum/num;
    }
    public double getAveragePositivePosition() {
        double sum = 0;
        double num = 1;
        for (MidnightMotor motor : motors) {
            sum += Math.abs(motor.getCurrentPosition());
            num++;
        }
        return sum/num;
    }
    public double getVelocity(){
        double i = 0;
        double rate = 0;
        for (MidnightMotor masqMotor: motors){
            rate += masqMotor.getVelocity();
            i++;
        }
        return rate/i;
    }
    public boolean isBusy() {
        boolean isBusy = false;
        for (MidnightMotor masqMotor: motors)
            isBusy = masqMotor.isBusy();
        return isBusy;
    }
    public double getCurrentPosition() {
        int total = 0;
        for (MidnightMotor m : motors) total += m.getCurrentPosition();
        return total / numMotors;
    }
    public double getAbsolutePosition ()  {
        int total = 0;
        for (MidnightMotor m : motors) total += m.getAbsolutePosition();
        return total / numMotors;
    }

    public double getKp() {
        return kp;
    }

    public double getKi() {
        return ki;
    }

    public double getKd() {
        return kd;
    }

    public MidnightMotorModel getEncoder() {
        return encoder;
    }
    public void setWheelDiameter(double diameter) {
        for (MidnightMotor motor : motors) {
            motor.encoder.setWheelDiameter(diameter);
        }
    }

    public void setEncoder(MidnightMotorModel encoder) {
        this.encoder = encoder;
    }

    public String getName() {
        return systemName;
    }
    public String[] getDash() {
        return new String[]{ "Current Position" + getCurrentPosition()};
    }
}