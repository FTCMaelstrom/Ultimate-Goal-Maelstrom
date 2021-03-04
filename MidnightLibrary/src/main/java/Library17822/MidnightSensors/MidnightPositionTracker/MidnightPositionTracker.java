package Library17822.MidnightSensors.MidnightPositionTracker;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

import Library17822.MidnightSensors.MidnightAdafruitIMU;
import Library17822.MidnightSensors.MidnightPositionTracker.MidnightDeadwheel.Measurement;

/**
 * Created by Archishmaan Peyyety on 2020-01-08.
 * Project: MasqLib
 */
public class MidnightPositionTracker {
    private List<MidnightDeadwheel> wheels = new ArrayList<>();
    private double globalX, globalY, prevHeading;
    private MidnightAdafruitIMU imu;
    private int xWheels, yWheels;

    public MidnightPositionTracker(HardwareMap hardwareMap) {
        imu = new MidnightAdafruitIMU("imu", hardwareMap);
        prevHeading = imu.getAbsoluteHeading();
        reset();
    }
    public MidnightPositionTracker(String imuName, HardwareMap hardwareMap) {
        imu = new MidnightAdafruitIMU(imuName, hardwareMap);
        reset();
    }

    public void updateSystem() {
        double heading = Math.toRadians(imu.getRelativeYaw());
        double dX, dY, dH, angularComponentX, angularComponentY;
        dX = dY = dH = angularComponentY = angularComponentX = 0;
        if (getWheelsType(Measurement.X).size() > 1) {

        } else {

        }
        if (getWheelsType(Measurement.Y).size() > 1) {
            double dW1 = getWheelsType(Measurement.Y).get(1).getVelocity();
            double dW2 = getWheelsType(Measurement.Y).get(2).getVelocity();
        } else {

        }
        double dTranslationalX = dX - angularComponentX;
        double dTranslationalY = dY + angularComponentY;
        double dGlobalX = dTranslationalX * Math.cos(heading) - dTranslationalY * Math.sin(heading);
        double dGlobalY = dTranslationalX * Math.sin(heading) + dTranslationalY * Math.cos(heading);
        globalX += dGlobalX;
        globalY += dGlobalY;
    }

    public void reset() {
        for (MidnightDeadwheel midnightDeadwheel : wheels) {
            midnightDeadwheel.reset();
        }
        imu.reset();
    }
    public double getGlobalX() {
        return globalX;
    }

    public double getGlobalY() {
        return globalY;
    }

    public List<MidnightDeadwheel> getWheelsType(Measurement m) {
        List<MidnightDeadwheel> deadwheels = new ArrayList<>();
        for (MidnightDeadwheel midnightDeadwheel : wheels) {
            if (midnightDeadwheel.getMeasurement() == m) deadwheels.add(midnightDeadwheel);
        }
        return deadwheels;
    }

    public void addWheel(MidnightDeadwheel deadwheel) {
        if (deadwheel.getMeasurement() == MidnightDeadwheel.Measurement.X) xWheels++;
        else yWheels++;
        wheels.add(deadwheel);
    }
}
