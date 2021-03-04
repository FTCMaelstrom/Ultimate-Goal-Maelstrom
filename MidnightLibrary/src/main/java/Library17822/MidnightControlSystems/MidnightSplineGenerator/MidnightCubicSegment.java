package Library17822.MidnightControlSystems.MidnightSplineGenerator;

public class MidnightCubicSegment {
    double a, b, c, d;

    public MidnightCubicSegment(double a, double b, double c, double d) {
        this.a = a;
        this.b = b;
        this.c = c;
        this.d = d;
    }

    public double getA() {
        return a;
    }

    public void setA(double a) {
        this.a = a;
    }

    public double getB() {
        return b;
    }

    public void setB(double b) {
        this.b = b;
    }

    public double getC() {
        return c;
    }

    public void setC(double c) {
        this.c = c;
    }

    public double getD() {
        return d;
    }

    public void setD(double d) {
        this.d = d;
    }

    public double compute(double t) {
        return (a * Math.pow(t, 3)) + (b * Math.pow(t, 2)) + (c * t) + d;
    }
}
