package MidnightLibrary.MidnightMath;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import MidnightLibrary.MidnightResources.MidnightClock;

import static com.qualcomm.robotcore.util.Range.clip;

/**
 * Created by Archish on 4/9/18.
 */

public class MidnightPIDController {
    private MidnightIntegrator integrator = new MidnightIntegrator();
    private double kp = 0;
    private double ki = 0;
    private double kd = 0;
    private double prevError = 0;
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
    public MidnightPIDController() {}

    public double getOutput (double error) {
        double timeChange = clock.seconds();
        double derivative = (error - prevError) / timeChange;
        double integral = integrator.getIntegral(error);
        clock.reset();
        prevError = error;
        return clip((error * kp) +
                (ki * integral) +
                (kd * derivative), -1, 1);
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