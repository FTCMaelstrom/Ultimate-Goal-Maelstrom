package Library17822.MidnightResources.MidnightMath;

import Library17822.MidnightResources.MidnightHelpers.MidnightHardware;

/**
 * Created by Archishmaan Peyyety on 8/13/18.
 * Project: MasqLib
 */

public class MidnightVector implements MidnightHardware {
    private double x;
    private double y;
    private String name;
    public MidnightVector(double x, double y, String name) {
        this.x = x;
        this.y = y;
        this.name = name;
    }

    public MidnightVector(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public MidnightPoint toPoint() {
        return new MidnightPoint(getX(), getY());
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public MidnightVector unitVector () {
        return new MidnightVector(getX()/getMagnitude(), getY()/getMagnitude());
    }

    public static MidnightVector multiply(double scalar, MidnightVector v) {
        return new MidnightVector(v.getX() * scalar, v.getY() * scalar);
    }

    public double getMagnitude () {
        return Math.hypot(getX(), getY());
    }

    public double dotProduct(MidnightVector v) {
        return (this.getX() * v.getX()) + (this.getY() * v.getY());
    }

    public double angleRad(MidnightVector v) {
        return (Math.acos(dotProduct(v) / (v.getMagnitude() * this.getMagnitude())));
    }
    public double angleDeg(MidnightVector v) {
        double deg = Math.toDegrees(Math.acos(dotProduct(v) / (v.getMagnitude() * this.getMagnitude())));
        if (Double.isNaN(deg)) return 0;
        return deg;
    }

    public double angleTan(MidnightVector v) {
        double atan1 = Math.atan2(v.getY(), v.getX());
        double atan2 = Math.atan2(getY(), getX());
        return Math.toDegrees(atan2 - atan1);
    }

    public double distanceToVector(MidnightVector point) {
        return Math.hypot(point.getX() - getX(), point.getY() - getY());
    }

    public boolean equal(double radius, MidnightVector v) {
        return distanceToVector(v) < radius;
    }

    public MidnightVector displacement(MidnightVector v) {
        return new MidnightVector(v.getX() - getX(), v.getY() - getY());
    }

    public MidnightVector projectOnTo(MidnightVector v) {
        return multiply(dotProduct(v) / (v.getMagnitude() * v.getMagnitude()), v);
    }

    public double getDirection () {
        return Math.toDegrees(Math.atan(getY() / getX()));
    }

    public MidnightVector setName(String name) {
        this.name = name;
        return this;
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public String[] getDash() {
        return new String[]{
                "X: " + getX(),
                "Y: " + getY(),
        };
    }
}
