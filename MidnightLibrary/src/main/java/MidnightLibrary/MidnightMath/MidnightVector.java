package MidnightLibrary.MidnightMath;

import androidx.annotation.NonNull;

import MidnightLibrary.MidnightResources.MidnightHardware;

import static java.lang.Math.atan;
import static java.lang.Math.atan2;
import static java.lang.Math.hypot;
import static java.lang.Math.toDegrees;
import static java.util.Locale.US;

/**
 * Created by Archishmaan Peyyety on 8/13/18.
 * Project: MidnightLib
 */

public class MidnightVector {
    private double x;
    private double y;
    private String name;

    public MidnightVector(String name, double x, double y) {
        this.x = x;
        this.y = y;
        this.name = name;
    }

    public double getX() {return x;}
    public double getY() {return y;}
    public void setX(double x) {this.x = x;}
    public void setY(double y) {this.y = y;}

    public double getMagnitude () {return hypot(getX(), getY());}
    public double getDirection () {return toDegrees(atan(getY() / getX()));}
    public MidnightVector unitVector () {return new MidnightVector(name, getX()/getMagnitude(), getY()/getMagnitude());}

    public MidnightVector multiply(double scalar) {return new MidnightVector(name, getX() * scalar, getY() * scalar);}
    public MidnightVector displacement(MidnightVector v) {return new MidnightVector(name, v.getX() - getX(), v.getY() - getY());}
    public MidnightVector projectOnTo(MidnightVector v) {return v.unitVector().multiply(dotProduct(v.unitVector()));}
    public MidnightVector add(MidnightVector v) {return new MidnightVector(name, getX() + v.getX(), getY() + v.getY());}
    public double dotProduct(MidnightVector v) {return (this.getX() * v.getX()) + (this.getY() * v.getY());}
    public double angleTo(MidnightVector v) {return toDegrees(atan2(getY(), getX()) - atan2(v.getY(), v.getX()));}
    public double distanceToVector(MidnightVector v) {return hypot(v.getX() - getX(), v.getY() - getY());}
    public boolean equal(double radius, MidnightVector v) {return distanceToVector(v) < radius;}

    public MidnightVector setName(String name) {
        this.name = name;
        return this;
    }

    @NonNull
    @Override
    public String toString() {return String.format(US, "%s:\nX: %.2f\nY: %.2f", name, x, y);}

    //private double x;
    //private double y;
    //private String name;
    public MidnightVector(double x, double y, String name) {
        this.x = x;
        this.y = y;
        this.name = name;
    }

    public MidnightVector(double x, double y) {
        this.x = x;
        this.y = y;
    }


    public MidnightPoint toPoint() {
        return new MidnightPoint(getX(), getY());
    }



    public static MidnightVector multiply(double scalar, MidnightVector v) {
        return new MidnightVector(v.getX() * scalar, v.getY() * scalar);
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


}
