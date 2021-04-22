package MidnightLibrary.MidnightMath;

import androidx.annotation.NonNull;

import static java.util.Locale.US;

/**
 * Created by Archishmaan Peyyety on 2020-01-23.
 * Project: MidnightLib
 */

/*
 * Modified 4/20/21 9:06 PM by Amogh Mehta
 */

public class MidnightWayPoint {
    private double x, y, h, targetRadius = 1, modeSwitchRadius = 10, pointSwitchRadius = 10,
            minVelocity = 0.5, maxVelocity = 1, timeout = 2, lookAhead = 10,
            angularCorrectionSpeed = 0.08, driveCorrectionSpeed = 0.12;
    private String name = "WayPoint";
    private PointMode switchMode = PointMode.MECH;
    private Runnable onComplete = () -> {
    };

    public MidnightWayPoint() {
    }

    public MidnightWayPoint(double x, double y, double h) {
        this.x = x;
        this.y = y;
        this.h = h;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public MidnightWayPoint setPoint(double x, double y, double h) {
        this.x = x;
        this.y = y;
        this.h = h;
        return this;
    }

    public MidnightWayPoint setX(double x) {
        this.x = x;
        return this;
    }

    public MidnightWayPoint setY(double y) {
        this.y = y;
        return this;
    }

    public MidnightWayPoint setH(double h) {
        this.h = h;
        return this;
    }

    public MidnightWayPoint setTargetRadius(double targetRadius) {
        modeSwitchRadius += targetRadius - this.targetRadius;
        pointSwitchRadius += targetRadius - this.targetRadius;
        this.targetRadius = targetRadius;
        return this;
    }

    public MidnightWayPoint setModeSwitchRadius(double modeSwitchRadius) {
        this.modeSwitchRadius = modeSwitchRadius;
        return this;
    }

    public MidnightWayPoint setPointSwitchRadius(double pointSwitchRadius) {
        this.pointSwitchRadius = pointSwitchRadius;
        return this;
    }

    public MidnightWayPoint setMinVelocity(double minVelocity) {
        this.minVelocity = minVelocity;
        return this;
    }

    public double getH() {
        return h;
    }

    public MidnightWayPoint setMaxVelocity(double maxVelocity) {
        this.maxVelocity = maxVelocity;
        return this;
    }

    public MidnightWayPoint setTimeout(double timeout) {
        this.timeout = timeout;
        return this;
    }

    public MidnightWayPoint setLookAhead(double lookAhead) {
        this.lookAhead = lookAhead;
        return this;
    }

    public MidnightWayPoint setAngularCorrectionSpeed(double angularCorrectionSpeed) {
        this.angularCorrectionSpeed = angularCorrectionSpeed;
        return this;
    }

    public MidnightWayPoint setOnComplete(Runnable onComplete) {
        this.onComplete = onComplete;
        return this;
    }

    public double getTargetRadius() {
        return targetRadius;
    }

    public double getModeSwitchRadius() {
        return modeSwitchRadius;
    }

    public double getPointSwitchRadius() {
        return pointSwitchRadius;
    }

    public double getMinVelocity() {
        return minVelocity;
    }

    public double getMaxVelocity() {
        return maxVelocity;
    }

    public double getTimeout() {
        return timeout;
    }

    public double getLookAhead() {
        return lookAhead;
    }

    public double getAngularCorrectionSpeed() {
        return angularCorrectionSpeed;
    }

    public double getDriveCorrectionSpeed() {
        return driveCorrectionSpeed;
    }

    public MidnightWayPoint setDriveCorrectionSpeed(double driveCorrectionSpeed) {
        this.driveCorrectionSpeed = driveCorrectionSpeed;
        return this;
    }

    public PointMode getSwitchMode() {
        return switchMode;
    }

    public MidnightWayPoint setSwitchMode(PointMode switchMode) {
        this.switchMode = switchMode;
        return this;
    }

    public Runnable getOnComplete() {
        return onComplete;
    }

    public MidnightVector getPoint() {
        return new MidnightVector(name, x, y);
    }

    @NonNull
    @Override
    public String toString() {
        return String.format(US, "%s:\nX: %.2f\nY: %.2f\nH: %.1f\n", name, x, y, h);
    }

    public MidnightWayPoint setName(String name) {
        this.name = name;
        return this;
    }

    public enum PointMode {MECH, TANK, SWITCH}
}