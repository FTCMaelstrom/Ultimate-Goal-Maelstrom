package MidnightLibrary.MidnightMotor;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import MidnightLibrary.MidnightSensors.MidnightLimitSwitch;
import MidnightLibrary.MidnightSensors.MidnightTouchSensor;

import static java.util.Locale.US;

/**
 * MidnightMotorSystem That supports two or more motors and treats them as one
 */

public class MidnightMotorSystem {
    public MidnightMotor motor1 , motor2, motor3;
    public List<MidnightMotor> motors;
    public int numMotors;
    double kp, ki, kd;
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
    public MidnightMotorSystem(String name1,DcMotor.Direction d1, String name2, DcMotor.Direction d2, MidnightMotorModel encoder, HardwareMap hardwareMap) {
        motor1 = new MidnightMotor(name1, encoder, d1, hardwareMap);
        motor2 = new MidnightMotor(name2, encoder, d2, hardwareMap);
        motors = Arrays.asList(motor1, motor2);
        numMotors = 2;
    }
    public MidnightMotorSystem(String name, MidnightMotorModel MidnightMotorModel, DcMotor.Direction direction, HardwareMap hardwareMap) {
        motor1 = new MidnightMotor(name, MidnightMotorModel, direction, hardwareMap);
        motors = Collections.singletonList(motor1);
        numMotors = 1;
    }
    public void resetEncoders() {
        for (MidnightMotor MidnightMotor : motors)
            MidnightMotor.resetEncoder();
    }
    public void setKp(double kp){
        this.kp = kp;
        for (MidnightMotor MidnightMotor: motors) MidnightMotor.setKp(kp);
    }

    public void setKi(double ki) {
        this.ki = ki;
        for (MidnightMotor MidnightMotor: motors) {
            MidnightMotor.setKi(ki);
        }
    }

    public void setKd(double kd) {
        this.kd= kd;
        for (MidnightMotor MidnightMotor: motors) {
            MidnightMotor.setKd(kd);
        }
    }

    public double getPower() {
        double num = 0, sum = 0;
        for (MidnightMotor MidnightMotor: motors) {
            sum += Math.abs(MidnightMotor.getPower());
            num++;
        }
        return sum/num;
    }
    public double getInches () {
        double num = 0, sum = 0;
        for (MidnightMotor MidnightMotor : motors) {
            sum += MidnightMotor.getEncoder().getInches();
            num++;
        }
        return sum/num;
    }
    public void setMinPower(double power) {
        for (MidnightMotor MidnightMotor : motors) MidnightMotor.setMinPower(power);
    }
    public void setVelocity(double power) {
        for (MidnightMotor MidnightMotor : motors) MidnightMotor.setVelocity(power);
    }

    public void setPower(double power) {
        for (MidnightMotor MidnightMotor : motors) MidnightMotor.setPower(power);
    }
    public void setClosedLoop (boolean closedLoop) {
        for (MidnightMotor MidnightMotor : motors) {
            MidnightMotor.setClosedLoop(closedLoop);
        }
    }
    public void setLimits (MidnightLimitSwitch min, MidnightLimitSwitch max) {
        for (MidnightMotor MidnightMotor : motors) {
            MidnightMotor.setLimits(min, max);
        }
    }

    public double getVelocity(){
        double rate = 0;
        for (MidnightMotor MidnightMotor: motors)rate += MidnightMotor.getVelocity();
        return rate / numMotors;
    }

    public int getCurrentPosition() {
        int total = 0;
        for (MidnightMotor m : motors) total += m.getCurrentPosition();
        return total / numMotors;
    }
    public int getAbsolutePosition ()  {
        int total = 0;
        for (MidnightMotor m : motors) total += m.getAbsolutePosition();
        return total / numMotors;
    }

    public void startVelocityControl() {
        for(MidnightMotor motor : motors) motor.startVelocityControl();
    }

    public void runUsingEncoder() {
        for(MidnightMotor motor : motors) motor.runUsingEncoder();
    }
    public void runWithoutEncoder() {
        for(MidnightMotor motor : motors) motor.runWithoutEncoder();
    }

    public String getName() {
        return systemName;
    }
    public String[] getDash() {
        return new String[]{ "Current Position" + getCurrentPosition()};
    }
}