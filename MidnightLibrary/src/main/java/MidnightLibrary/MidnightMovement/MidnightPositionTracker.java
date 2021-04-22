package MidnightLibrary.MidnightMovement;

import com.qualcomm.robotcore.hardware.HardwareMap;

import MidnightLibrary.MidnightMotor.MidnightMotor;
import MidnightLibrary.MidnightResources.MidnightClock;
import MidnightLibrary.MidnightResources.MidnightDashBoard;
import MidnightLibrary.MidnightResources.MidnightHardware;
import MidnightLibrary.MidnightResources.MidnightUtils;
import MidnightLibrary.MidnightSensors.MidnightAdafruitIMU;

import static MidnightLibrary.MidnightResources.MidnightClock.Resolution.SECONDS;
import static MidnightLibrary.MidnightResources.MidnightUtils.adjustAngle;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;

/**
 * Created by Amogh Mehta
 * Project: FtcRobotController_Ultimate-Goal_prod2
 * Last Modified: 3/15/21 1:28 PM
 * Last Updated: 3/16/21 3:48 PM
 **/

public class MidnightPositionTracker implements MidnightHardware {
    private final MidnightDashBoard dash = MidnightDashBoard.getDash();
    public MidnightAdafruitIMU imu;
    private MidnightMotor xSystem;
    private MidnightMotor yLSystem;
    private MidnightMotor yRSystem;
    private MidnightMotor ySystem;
    private double prevHeading, xDrift, yDrift;
    private double globalX, globalY, prevX, prevY, prevYR, prevYL, xRadius, yRadius, trackWidth;
    private DeadWheelPosition position;

    public MidnightPositionTracker(MidnightMotor xSystem, MidnightMotor yLSystem, MidnightMotor yRSystem, HardwareMap hardwareMap) {
        this.xSystem = xSystem;
        this.yLSystem = yLSystem;
        this.yRSystem = yRSystem;
        imu = new MidnightAdafruitIMU("imu", hardwareMap);
        prevHeading = imu.getAbsoluteHeading();
        MidnightUtils.setTracker(this);
        reset();
    }

    public MidnightPositionTracker(MidnightMotor xSystem, MidnightMotor ySystem, HardwareMap hardwareMap) {
        this.xSystem = xSystem;
        this.ySystem = ySystem;
        imu = new MidnightAdafruitIMU("imu", hardwareMap);
        prevHeading = imu.getAbsoluteHeading();
        MidnightUtils.setTracker(this);
        reset();
    }

    public MidnightPositionTracker(HardwareMap hardwareMap) {
        imu = new MidnightAdafruitIMU("imu", hardwareMap);
        MidnightUtils.setTracker(this);
        imu.reset();
    }

    public double getHeading() {
        return imu.getRelativeYaw();
    }

    public void updateSystem() {
        switch (position) {
            case BOTH_CENTER:
                bothCenter();
                break;
            case BOTH_PERPENDICULAR:
                bothPerpendicular();
                break;
            case THREE:
                three();
                break;
        }
    }

    public void updateOverTime(double time) {
        MidnightClock clock = new MidnightClock();
        while (clock.hasNotPassed(time, SECONDS)) {
            updateSystem();
            dash.create(this);
            dash.update();
        }
    }

    public void reset() {
        if (xSystem != null) {
            xSystem.resetEncoder();
            xSystem.setWheelDiameter(2);
        }
        if (ySystem != null) {
            ySystem.resetEncoder();
            ySystem.setWheelDiameter(2);
        }
        if (yLSystem != null && yRSystem != null) {
            yLSystem.resetEncoder();
            yRSystem.resetEncoder();
            yLSystem.setWheelDiameter(2);
            yRSystem.setWheelDiameter(2);
        }

        imu.reset();

        globalX = 0;
        globalY = 0;
    }

    private void bothCenter() {
        double deltaX = (xSystem.getInches() - prevX);
        double deltaY = (ySystem.getInches() - prevY);
        double heading = toRadians(getHeading());
        double x = deltaX * cos(heading) - deltaY * sin(heading);
        double y = deltaX * sin(heading) + deltaY * cos(heading);
        globalX += x;
        globalY += y;
        prevY = ySystem.getInches();
        prevX = xSystem.getInches();
    }

    private void bothPerpendicular() {
        double heading = toRadians(getHeading());
        double xPosition = xSystem.getInches();
        double yPosition = ySystem.getInches();
        double dH = getDHeading();
        double dX = xPosition - prevX;
        prevX = xPosition;
        double dY = yPosition - prevY;
        prevY = yPosition;
        double angularComponentY = yRadius * dH;
        double angularComponentX = xRadius * dH;
        double dTranslationalX = dX + angularComponentX;
        double dTranslationalY = dY - angularComponentY;
        double dGlobalX = dTranslationalX * cos(heading) + dTranslationalY * sin(heading);
        double dGlobalY = dTranslationalY * cos(heading) - dTranslationalX * sin(heading);
        globalX += dGlobalX;
        globalY += dGlobalY;
    }

    private void three() {
        double heading = toRadians(getHeading());
        double xPosition = xSystem.getInches();
        double yLPosition = yLSystem.getInches();
        double yRPosition = yRSystem.getInches();
        double dX = xPosition - prevX;
        prevX = xPosition;
        double dYR = yRPosition - prevYR;
        prevYR = yRPosition;
        double dYL = yLPosition - prevYL;
        prevYL = yLPosition;
        double dH = (dYR - dYL) / trackWidth;
        double dTranslationalY = (dYR + dYL) / 2;
        double angularComponentX = xRadius * dH;
        double dTranslationalX = dX - angularComponentX;
        double dGlobalX = dTranslationalX * cos(heading) + dTranslationalY * sin(heading);
        double dGlobalY = dTranslationalY * cos(heading) - dTranslationalX * sin(heading);
        globalX += dGlobalX;
        globalY += dGlobalY;
    }

    public double getDHeading() {
        double current = toRadians(getHeading());
        double change = (current - prevHeading);
        prevHeading = current;
        return adjustAngle(change, RADIANS);
    }

    public double getGlobalX() {
        return globalX + xDrift;
    }

    public double getGlobalY() {
        return globalY + yDrift;
    }

    public void setXRadius(double xRadius) {
        this.xRadius = xRadius;
    }

    public void setYRadius(double yRadius) {
        this.yRadius = yRadius;
    }

    public void setTrackWidth(double trackWidth) {
        this.trackWidth = trackWidth;
    }

    public void setPosition(MidnightPositionTracker.DeadWheelPosition position) {
        this.position = position;
    }

    @Override
    public String getName() {
        return "Tracker";
    }

    @Override
    public String[] getDash() {
        return new String[]{
                "GlobalX: " + globalX,
                "GlobalY: " + globalY,
                "Heading: " + getHeading(),
        };
    }

    public enum DeadWheelPosition {
        BOTH_CENTER, BOTH_PERPENDICULAR, THREE
    }
}
