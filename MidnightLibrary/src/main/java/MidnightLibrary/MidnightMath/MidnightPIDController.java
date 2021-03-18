package MidnightLibrary.MidnightMath;

import com.qualcomm.robotcore.util.Range;

import MidnightLibrary.MidnightResources.MidnightClock;

/**
 * Created by Archish on 4/9/18.
 */

public class MidnightPIDController {
    private MidnightIntegrator integrator = new MidnightIntegrator();
    private double kp;
    private double ki = 0;
    private double kd = 0;
    private double prevError = 0;
    private double prevD = 0;
    private double deriv, timeChange = 0;
    private MidnightClock clock = new MidnightClock();

    public MidnightPIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }
    public MidnightPIDController(double kp, double ki) {
        this.kp = kp;
        this.ki = ki;
    }
    public MidnightPIDController(double kp) {
        this.kp = kp;
    }

    //For testing
    public double getOutput (double error, double timeChange) {
        this.timeChange = timeChange;
        clock.reset();
        deriv = (error - prevError) / timeChange;
        prevError = error;
        prevD = deriv;
        return Range.clip((error * kp) +
                (ki * integrator.getIntegral(error, timeChange)) +
                (kd * deriv), -1, 1);
    }

    //For normal use
    public double getOutput (double error) {
        return getOutput(error,clock.seconds());
    }

    public double[] getConstants() {
        return new double[]{kp, ki, kd};
    }

    public void setConstants(int[] constants) {
        this.kp = constants[0];
        this.ki = constants[1];
        this.kd = constants[2];
    }

    public void setConstants(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
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

    public MidnightClock getClock() {
        return clock;
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

}