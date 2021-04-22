package MidnightLibrary.MidnightMovement;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import MidnightLibrary.MidnightAuxiliary.MidnightUtils;
import MidnightLibrary.MidnightSensors.MidnightEncoder;
import MidnightLibrary.MidnightSensors.MidnightLimitSwitch;

import static MidnightLibrary.MidnightAuxiliary.MidnightUtils.opModeIsActive;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT;
import static java.lang.Math.abs;
import static java.lang.System.nanoTime;

/**
 * This is a custom motor that includes stall detection and telemetry
 */

public class MidnightMotor {
    public DcMotor motor;
    public MidnightEncoder encoder;
    public double rpmIntegral = 0;
    public double rpmDerivative = 0;
    private double minPower = 0;
    private boolean stallDetection = false;
    private final String nameMotor;
    private double targetPower;
    private boolean velocityControlState = false;
    private double kp = 0.001, ki = 0, kd = 0;
    private double prevPos = 0;
    private boolean stalled = false;
    private double previousTime = 0;
    private double motorPower;
    private double currentMax, currentMin;
    private double rpmPreviousError = 0;
    private int stalledRPMThreshold = 10;
    private boolean reversedEncoder = false;
    private Runnable
            stallAction,
            unStalledAction;
    private double minPosition, maxPosition;
    private boolean
            limitDetection,
            positionDetection,
            halfDetectionMin = false,
            halfDetectionMax = false,
            closedLoop = false;
    private MidnightLimitSwitch minLim, maxLim = null;

    public MidnightMotor(String name, MidnightMotorModel model, HardwareMap hardwareMap) {
        limitDetection = positionDetection = false;
        this.nameMotor = name;
        motor = hardwareMap.get(DcMotor.class, name);
        encoder = new MidnightEncoder(this, model);
    }

    public MidnightMotor(String name, MidnightMotorModel model, DcMotor.Direction direction, HardwareMap hardwareMap) {
        limitDetection = positionDetection = false;
        this.nameMotor = name;
        motor = hardwareMap.dcMotor.get(name);
        motor.setDirection(direction);
        encoder = new MidnightEncoder(this, model);
    }

    public void setLimits(MidnightLimitSwitch min, MidnightLimitSwitch max) {
        maxLim = max;
        minLim = min;
        limitDetection = true;
    }

    public MidnightMotor setLimit(MidnightLimitSwitch min) {
        minLim = min;
        maxLim = null;
        limitDetection = true;
        return this;
    }

    public MidnightMotor setPositionLimits(double min, double max) {
        minPosition = min;
        maxPosition = max;
        positionDetection = true;
        return this;
    }

    public MidnightMotor setHalfLimits(MidnightLimitSwitch min, double max) {
        maxPosition = max;
        minLim = min;
        halfDetectionMin = true;
        return this;
    }

    public MidnightMotor setHalfLimits(double min, MidnightLimitSwitch max) {
        minPosition = min;
        maxLim = max;
        halfDetectionMax = true;
        return this;
    }

    public MidnightMotor setPositionLimit(double min) {
        minPosition = min;
        positionDetection = true;
        return this;
    }

    public void runWithoutEncoder() {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(FLOAT);
    }

    public void runUsingEncoder() {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(BRAKE);
    }

    public void resetEncoder() {
        encoder.resetEncoder();
    }

    boolean isBusy() {
        return motor.isBusy();
    }

    public double getCurrentPosition() {
        return encoder.getRelativePosition();
    }

    public double getAbsolutePosition() {
        if (reversedEncoder) return -motor.getCurrentPosition();
        return motor.getCurrentPosition();
    }

    public double getVelocity() {
        double deltaPosition = getCurrentPosition() - prevPos;
        previousTime = nanoTime();
        double tChange = (nanoTime() - previousTime) / 1e9;
        prevPos = getCurrentPosition();
        double rate = deltaPosition / tChange;
        rate = (rate * 60) / encoder.getClicksPerRotation() / encoder.getRPM();
        return rate;
    }

    public void setVelocity(double power) {
        targetPower = power;
        motorPower = calculateVelocityCorrection(power);
        if (!closedLoop) motorPower = power;
        double currentZero;
        if (limitDetection) {
            if (minLim != null && minLim.isPressed() && power < 0 ||
                    maxLim != null && maxLim.isPressed() && power > 0)
                motorPower = 0;
            else if (minLim != null && minLim.isPressed()
                    && power < 0 && maxLim == null)
                motorPower = 0;
        } else if (positionDetection) {
            if ((motor.getCurrentPosition() < minPosition && power < 0) ||
                    (motor.getCurrentPosition() > maxPosition && power > 0))
                motorPower = 0;
            else if (motor.getCurrentPosition() < minPosition && power < 0)
                motorPower = 0;
        } else if (halfDetectionMin) {
            if (minLim.isPressed()) {
                currentZero = motor.getCurrentPosition();
                currentMax = currentZero + maxPosition;
            }
            if (minLim != null && minLim.isPressed() && power < 0) motorPower = 0;
            else if (motor.getCurrentPosition() > currentMax && power > 0) motorPower = 0;
        } else if (halfDetectionMax) {
            if (maxLim.isPressed()) {
                currentZero = motor.getCurrentPosition();
                currentMin = currentZero - minPosition;
            }
            if (maxLim != null && maxLim.isPressed() && power > 0) motorPower = 0;
            else if (motor.getCurrentPosition() < currentMin && power < 0) motorPower = 0;
        }
        if (abs(motorPower) < minPower && minPower != 0) motorPower = 0;
        motor.setPower(motorPower);
    }

    public double calculateVelocityCorrection(double power) {
        double tChange = (nanoTime() - previousTime) / 1e9;
        double error = encoder.getRPM() * (power - getVelocity());
        rpmIntegral += error * tChange;
        rpmDerivative = (error - rpmPreviousError) / tChange;
        double p = error * kp;
        double i = rpmIntegral * ki;
        double d = rpmDerivative * kd;
        double motorPower = power + (p + i + d);
        rpmPreviousError = error;
        previousTime = nanoTime();
        return motorPower;
    }

    public void setVelocityControlState(boolean velocityControlState) {
        this.velocityControlState = velocityControlState;
    }

    public void startVelocityControl() {
        setVelocityControlState(true);
        Runnable velocityControl = () -> {
            while (opModeIsActive() && velocityControlState) setVelocity(targetPower);
        };
        Thread velocityThread = new Thread(velocityControl);
        velocityThread.start();
    }

    private boolean getStalled() {
        return abs(getVelocity() * encoder.getRPM()) < stalledRPMThreshold;
    }

    public void setStalledAction(Runnable action) {
        stallAction = action;
    }

    public void setUnStalledAction(Runnable action) {
        unStalledAction = action;
    }

    private boolean getStallDetection() {
        return stallDetection;
    }

    public void setStallDetection(boolean bool) {
        stallDetection = bool;
    }

    public synchronized boolean isStalled() {
        return stalled;
    }

    public void setStalledRPMThreshold(int stalledRPMThreshold) {
        this.stalledRPMThreshold = stalledRPMThreshold;
    }

    public void enableStallDetection() {
        setStallDetection(true);
        Runnable mainRunnable = () -> {
            while (opModeIsActive()) {
                stalled = getStalled();
                if (getStallDetection()) {
                    if (stalled) stallAction.run();
                    else unStalledAction.run();
                }
                MidnightUtils.sleep(100);
            }
        };
        Thread thread = new Thread(mainRunnable);
        thread.start();
    }

    public double getPower() {
        return motorPower;
    }

    public void setPower(double power) {
        power = Range.clip(power, -1, 1);
        motorPower = power;
        motor.setPower(power);
    }

    public MidnightEncoder getEncoder() {
        return encoder;
    }

    public void setKp(double kp) {
        this.kp = kp;
    }

    public void setKi(double ki) {
        this.ki = ki;
    }

    public void setKd(double kd) {
        this.kd = kd;
    }

    public DcMotorController getController() {
        return motor.getController();
    }

    public int getPortNumber() {
        return motor.getPortNumber();
    }

    public void setMotorModel(MidnightMotorModel model) {
        encoder.setModel(model);
    }

    public boolean isClosedLoop() {
        return closedLoop;
    }

    public void setClosedLoop(boolean closedLoop) {
        this.closedLoop = closedLoop;
    }

    public double getMinPower() {
        return minPower;
    }

    public void setMinPower(double power) {
        minPower = power;
    }

    public void setWheelDiameter(double diameter) {
        encoder.setWheelDiameter(diameter);
    }

    public double getInches() {
        return encoder.getInches();
    }

    public double getTargetPower() {
        return targetPower;
    }

    public void reverseEncoder() {
        reversedEncoder = !reversedEncoder;
    }

    public void setDirection(DcMotor.Direction direction) {
        motor.setDirection(direction);
    }

    public String getName() {
        return nameMotor;
    }

    public String[] getDash() {
        return new String[]{
                "Current Position: " + getCurrentPosition(),
                "Velocity: " + getVelocity()};
    }

    public void setStopMotor() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void brakeMode() {
        motor.setZeroPowerBehavior(BRAKE);
    }


    public void runWithoutEncoderNoZeroPowerSet() {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void accessoryMotorConditioner() {
        setStopMotor();
        resetEncoder();
        runWithoutEncoderNoZeroPowerSet();
        brakeMode();
    }
}