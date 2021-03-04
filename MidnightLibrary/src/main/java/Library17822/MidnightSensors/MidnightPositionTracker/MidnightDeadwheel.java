package Library17822.MidnightSensors.MidnightPositionTracker;

import Library17822.MidnightMotors.MidnightMotor;

/**
 * Created by Archishmaan Peyyety on 2020-01-08.
 * Project: MasqLib
 */
public class MidnightDeadwheel {
    private MidnightMotor motor;
    private WheelPosition wheelPosition;
    private Measurement measurement;
    private double latestPosition, latestTime, prevPosition, prevTime, radius;
    public enum WheelPosition {
        TOP, BOTTOM, LEFT, RIGHT, CENTER
    }
    public enum Measurement {
        X, Y
    }

    public MidnightDeadwheel(MidnightMotor motor, WheelPosition wheelPosition, Measurement measurement, double radius) {
        this.motor = motor;
        this.wheelPosition = wheelPosition;
        this.measurement = measurement;
        this.radius = radius;
    }

    public void reset(){
        motor.resetEncoder();
    }
    public double getPosition() {
        return motor.getCurrentPosition();
    }

    public WheelPosition getWheelPosition() {
        return wheelPosition;
    }

    public double getRadius() {
        return radius;
    }

    public Measurement getMeasurement() {
        return measurement;
    }

    public double getInches() {
        return 0;
    }

    public double getChange() {
        latestPosition = getPosition();
        double change = latestPosition - prevPosition;
        prevPosition = latestPosition;
        return change;
    }
    public double getVelocity() {
        latestTime = System.nanoTime();
        double change = latestTime - prevTime;
        prevTime = latestTime;
        return getChange() / change;
    }
}
