package MidnightLibrary.MidnightMath;

import MidnightLibrary.MidnightResources.MidnightHardware;

/**
 * Created by Archishmaan Peyyety on 2020-01-23.
 * Project: MidnightLib
 */

public class MidnightWayPoint implements MidnightHardware {
    private double x, y, h, targetRadius = 1, modeSwitchRadius = 10, pointSwitchRadius = 10,
            minVelocity = 0.5, maxVelocity = 1, timeout = 2, lookAhead = 10,
            angularCorrectionSpeed = 0.08, speedBias = 0.5, driveCorrectionSpeed = 0.07,
            turnRadius = 5;


    private String name;
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

    public MidnightWayPoint setPoint(double x, double y, double h) {
        this.x = x;
        this.y = y;
        this.h = h;
        return this;
    }

    public double getX() {
        return x;
    }

    public MidnightWayPoint setX(double x) {
        this.x = x;
        return this;
    }

    public double getY() {
        return y;
    }

    public MidnightWayPoint setY(double y) {
        this.y = y;
        return this;
    }

    public double getH() {
        return h;
    }

    public MidnightWayPoint setH(double h) {
        this.h = h;
        return this;
    }

    public double getTargetRadius() {
        return targetRadius;
    }

    public MidnightWayPoint setTargetRadius(double targetRadius) {
        modeSwitchRadius += targetRadius - this.targetRadius;
        pointSwitchRadius += targetRadius - this.targetRadius;
        this.targetRadius = targetRadius;
        return this;
    }

    public double getModeSwitchRadius() {
        return modeSwitchRadius;
    }

    public MidnightWayPoint setModeSwitchRadius(double modeSwitchRadius) {
        this.modeSwitchRadius = modeSwitchRadius;
        return this;
    }

    public double getPointSwitchRadius() {
        return pointSwitchRadius;
    }

    public MidnightWayPoint setPointSwitchRadius(double pointSwitchRadius) {
        this.pointSwitchRadius = pointSwitchRadius;
        return this;
    }

    public double getMinVelocity() {
        return minVelocity;
    }

    public MidnightWayPoint setMinVelocity(double minVelocity) {
        this.minVelocity = minVelocity;
        return this;
    }

    public double getMaxVelocity() {
        return maxVelocity;
    }

    public MidnightWayPoint setMaxVelocity(double maxVelocity) {
        this.maxVelocity = maxVelocity;
        return this;
    }

    public double getTimeout() {
        return timeout;
    }

    public MidnightWayPoint setTimeout(double timeout) {
        this.timeout = timeout;
        return this;
    }

    public double getLookAhead() {
        return lookAhead;
    }

    public MidnightWayPoint setLookAhead(double lookAhead) {
        this.lookAhead = lookAhead;
        return this;
    }

    public double getAngularCorrectionSpeed() {
        return angularCorrectionSpeed;
    }

    public MidnightWayPoint setAngularCorrectionSpeed(double angularCorrectionSpeed) {
        this.angularCorrectionSpeed = angularCorrectionSpeed;
        return this;
    }

    public PointMode getSwitchMode() {
        return switchMode;
    }

    public MidnightWayPoint setSwitchMode(PointMode switchMode) {
        this.switchMode = switchMode;
        return this;
    }

    public double getDriveCorrectionSpeed() {
        return driveCorrectionSpeed;
    }

    public MidnightWayPoint setDriveCorrectionSpeed(double driveCorrectionSpeed) {
        this.driveCorrectionSpeed = driveCorrectionSpeed;
        return this;
    }

    public double getSpeedBias() {
        return speedBias;
    }

    public MidnightWayPoint setSpeedBias(double speedBias) {
        this.speedBias = speedBias;
        return this;
    }

    public Runnable getOnComplete() {
        return onComplete;
    }

    public MidnightWayPoint setOnComplete(Runnable onComplete) {
        this.onComplete = onComplete;
        return this;
    }

    public double getTurnRadius() {
        return turnRadius;
    }

    public void setTurnRadius(double turnRadius) {
        this.turnRadius = turnRadius;
    }

    public MidnightVector getPoint() {
        return new MidnightVector(x, y);
    }

    public MidnightWayPoint setPoint(MidnightPoint p) {
        this.x = p.getX();
        this.y = p.getY();
        this.h = p.getH();
        return this;
    }

    @Override
    public String getName() {
        return name;
    }

    public MidnightWayPoint setName(String name) {
        this.name = name;
        return this;
    }

    @Override
    public String[] getDash() {
        return new String[0];
    }

    public enum PointMode {
        MECH, TANK, SWITCH
    }
}