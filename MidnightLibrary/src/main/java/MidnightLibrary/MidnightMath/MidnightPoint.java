package MidnightLibrary.MidnightMath;

import MidnightLibrary.MidnightResources.MidnightHardware;

/**
 * Created by Archishmaan Peyyety on 10/6/18.
 * Project: MidnightLib
 */

public class MidnightPoint implements MidnightHardware {
    private double x, y, h;


    public MidnightPoint(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public MidnightPoint(double x, double y, double h) {
        this.x = x;
        this.y = y;
        this.h = h;
    }

    public double getH() {
        return h;
    }

    public void setH(double h) {
        this.h = h;
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public MidnightVector toVector() {
        return new MidnightVector(getX(), getY());
    }

    @Override
    public String getName() {
        return "Point";
    }

    @Override
    public String[] getDash() {
        return new String[]{
                "X: " + getX(),
                "Y: " + getY()
        };
    }
}
