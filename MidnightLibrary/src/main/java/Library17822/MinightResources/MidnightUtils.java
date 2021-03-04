package Library17822.MinightResources;

import android.graphics.Point;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.opencv.core.Rect;

import java.util.Locale;

import Library17822.MidnightControlSystems.MidnightPID.MidnightPIDController;
import Library17822.MidnightSensors.MidnightClock;
import Library17822.MinightResources.MasqMath.MidnightVector;
import Library17822.MidnightWrappers.MidnightLinearOpMode;


/**
 * Created by Archish on 10/16/17.
 */

public class MidnightUtils {
    private static MidnightLinearOpMode linearOpMode;
    public static final double DEFAULT_SLEEP_TIME = 0.5;
    public static final double DEFAULT_TIMEOUT = 2;
    public static final double DEFAULT_SPEED_MULTIPLIER = Math.sqrt(2);
    public static final double DEFAULT_TURN_MULTIPLIER = 1;
    public static final double DEFAULT_TOLERANCE = 2;
    public static final double STALL_POWER = 0.1;
    public static final double ODS_WHITE = 0.7, ODS_BLACK = 0.3;
    public static final String VUFORIA_KEY = "Ac5sAIr/////AAABmeUEovYOek9pkuVkMLDtWVGIkr+aSwnxHoPcO" +
            "Wo55EZxWMznvy9o+sR4uE8cUkHfJ2QywQNfK9SgCKSgjjRXD1lJvl3xiT0ddSjfE8JT9NMvGojoFG3nkaQP+Sq" +
            "MGTgr25mUnTM3Y7v5kcetBEF1+vIcQL28SnoWDfGGMQ9Yt9IHo/W/72s5qWMCJLSS7/8X+Scybt98htjPVAOPI" +
            "dcudmKVGUMIK5ajH8riMC/2i80n57oBV3YmEYFKq0kIl1/Yf0KP3Hre8pA2les4GgriDHZBmp/E/ixOo1H924+" +
            "DrFzuLwkk7gs7kk4Jrdp1+jqrxPBJdr8MjYjtXjW+epFt1lcvIlP/4MK44iEH9AMQXYD9";

    public static MidnightPIDController turnController;
    public static MidnightPIDController xySpeedController;
    public static MidnightPIDController xSpeedController = new MidnightPIDController(0.04);
    public static MidnightPIDController ySpeedController = new MidnightPIDController(0.07);
    public static MidnightPIDController xyAngleController;
    public static MidnightPIDController driveController;
    public static MidnightPIDController velocityTeleController;
    public static MidnightPIDController velocityAutoController;
    public static MidnightPIDController angleController;

    public enum  AngleUnits{DEGREE, RADIAN}

    public static void sleep() {
        sleep(MidnightUtils.DEFAULT_SLEEP_TIME, MidnightClock.Resolution.SECONDS);
    }
    public static void setLinearOpMode(MidnightLinearOpMode pLinearOpMode) {
        linearOpMode = pLinearOpMode;
    }

    public static double adjustAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    public static double adjustAngleRad(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle <= -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public static boolean tolerance(double value1, double value2, double tolerance) {
        return Math.abs(value1 - value2) < tolerance;
    }
    public static Telemetry getTelemetry() {
        return linearOpMode.telemetry;
    }
    public static MidnightLinearOpMode getLinearOpMode () {
        return linearOpMode;
    }
    public static boolean opModeIsActive() {
        return linearOpMode.opModeIsActive();
    }
    public HardwareMap getHardwareMap() {
        return linearOpMode.hardwareMap;
    }

    public static double max(double... vals) {
        double max = Double.MIN_VALUE;
        for (double d: vals) if (max < d) max = d;
        return max;
    }
    public static double min(double... vals) {
        double min = Double.MAX_VALUE;
        for (double d: vals) if (min > d) min = d;
        return min;
    }
    public static double scaleNumber(double m, double currentMin, double currentMax, double newMin, double newMax) {
        return (((m - currentMin) * (newMax - newMin)) / (currentMax - currentMin)) + newMin;
    }
    public static double scaleNumber(double m, double newMin, double newMax) {
        return scaleNumber(m, 0, 1, newMin, newMax);
    }
    public static Double formatAngle(AngleUnit angleUnit, double angle) {
        return Double.valueOf(formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle)));
    }
    private static String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
    public static Point getCenterPoint(Rect rect) {
        return new Point(rect.x + rect.width/2, rect.y + rect.height/2);
    }
    public static void sleep(double time, MidnightClock.Resolution resolution) {
        try {
            Thread.sleep((long) ((time * resolution.multiplier) / MidnightClock.Resolution.MILLISECONDS.multiplier));
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    public static MidnightVector getLookAhead(MidnightVector initial, MidnightVector current, MidnightVector finalPos, double lookAhead) {
        MidnightVector pathDisplacement = initial.displacement(finalPos);
        MidnightVector untransformedProjection = new MidnightVector(
                current.projectOnTo(pathDisplacement).getX() - initial.getX(),
                current.projectOnTo(pathDisplacement).getY() - initial.getY()).projectOnTo(pathDisplacement);
        MidnightVector projection = new MidnightVector(
                untransformedProjection.getX() + initial.getX(),
                untransformedProjection.getY() + initial.getY());
        double theta = Math.atan2(pathDisplacement.getY(), pathDisplacement.getX());
        return new MidnightVector(
                projection.getX() + (lookAhead * Math.cos(theta)),
                projection.getY() + (lookAhead * Math.sin(theta)));
    }

}