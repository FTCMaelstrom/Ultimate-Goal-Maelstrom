package MidnightLibrary.MidnightAuxiliary;

import android.graphics.Point;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.opencv.core.Rect;

import java.util.Locale;

import MidnightLibrary.MidnightDrivetrain.MidnightPositionTracker;
import MidnightLibrary.MidnightMath.MidnightPIDController;
import MidnightLibrary.MidnightMath.MidnightVector;

import static java.lang.Double.valueOf;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;


/**
 * Created by Archish on 10/16/17.
 */

/*
 * Modified 4/20/21 9:06 PM by Amogh Mehta
 */

public class MidnightUtils {
    public static final long DEFAULT_SLEEP_TIME = 500;
    public static final double DEFAULT_TIMEOUT = 2;
    public static final double DEFAULT_SPEED_MULTIPLIER = Math.sqrt(2);
    public static final double BOOST_SPEED_MULTIPLIER = 2;
    public static final double DEFAULT_TURN_MULTIPLIER = 1;
    public static final double ODS_WHITE = 0.7, ODS_BLACK = 0.3;
    public static final String VUFORIA_KEY = "Ac5sAIr/////AAABmeUEovYOek9pkuVkMLDtWVGIkr+aSwnxHoPcO" +
            "Wo55EZxWMznvy9o+sR4uE8cUkHfJ2QywQNfK9SgCKSgjjRXD1lJvl3xiT0ddSjfE8JT9NMvGojoFG3nkaQP+Sq" +
            "MGTgr25mUnTM3Y7v5kcetBEF1+vIcQL28SnoWDfGGMQ9Yt9IHo/W/72s5qWMCJLSS7/8X+Scybt98htjPVAOPI" +
            "dcudmKVGUMIK5ajH8riMC/2i80n57oBV3YmEYFKq0kIl1/Yf0KP3Hre8pA2les4GgriDHZBmp/E/ixOo1H924+" +
            "DrFzuLwkk7gs7kk4Jrdp1+jqrxPBJdr8MjYjtXjW+epFt1lcvIlP/4MK44iEH9AMQXYD9";
    public static MidnightPIDController turnController;
    public static MidnightPIDController driveController;
    public static MidnightPIDController angleController;
    private static MidnightLinearOpMode linearOpMode;
    private static MidnightPositionTracker tracker;

    public static void sleep(long milliSeconds) {
        getLinearOpMode().sleep(milliSeconds);
    }

    public static void sleep() {
        sleep(DEFAULT_SLEEP_TIME);
    }

    public static MidnightPositionTracker getTracker() {
        return tracker;
    }

    public static void setTracker(MidnightPositionTracker positionTracker) {
        tracker = positionTracker;
    }

    public static double adjustAngle(double angle, AngleUnit angleUnit) {
        return angleUnit.normalize(angle);
    }

    public static double adjustAngle(double angle) {
        return adjustAngle(angle, DEGREES);
    }

    public static boolean tolerance(double value1, double value2, double tolerance) {
        return Math.abs(value1 - value2) < tolerance;
    }

    public static MidnightLinearOpMode getLinearOpMode() {
        return linearOpMode;
    }

    public static void setLinearOpMode(MidnightLinearOpMode opMode) {
        linearOpMode = opMode;
    }

    public static boolean opModeIsActive() {
        return linearOpMode.opModeIsActive();
    }

    public static HardwareMap getHardwareMap() {
        return linearOpMode.hardwareMap;
    }

    public static double max(double... vals) {
        double max = Double.MIN_VALUE;
        for (double d : vals) if (max < d) max = d;
        return max;
    }

    public static double min(double... vals) {
        double min = Double.MAX_VALUE;
        for (double d : vals) if (min > d) min = d;
        return min;
    }

    public static double scaleNumber(double m, double currentMin, double currentMax, double newMin, double newMax) {
        return (((m - currentMin) * (newMax - newMin)) / (currentMax - currentMin)) + newMin;
    }

    public static double scaleNumber(double m, double newMin, double newMax) {
        return scaleNumber(m, 0, 1, newMin, newMax);
    }

    public static Double formatAngle(AngleUnit angleUnit, double angle) {
        return valueOf(formatDegrees(DEGREES.fromUnit(angleUnit, angle)));
    }

    private static String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", DEGREES.normalize(degrees));
    }

    public static Point getCenterPoint(Rect rect) {
        return new Point(rect.x + rect.width / 2, rect.y + rect.height / 2);
    }

    public static MidnightVector getLookAhead(MidnightVector initial, MidnightVector current, MidnightVector finalPos, double lookAhead) {
        MidnightVector pathDisplacement = initial.displacement(finalPos);
        MidnightVector projection = new MidnightVector("Projection",
                current.projectOnTo(pathDisplacement).getX() - initial.getX(),
                current.projectOnTo(pathDisplacement).getY() - initial.getY()).projectOnTo(pathDisplacement).add(initial);
        double theta = atan2(pathDisplacement.getY(), pathDisplacement.getX());
        return new MidnightVector("Look Ahead", projection.getX() + (lookAhead * cos(theta)),
                projection.getY() + (lookAhead * sin(theta)));
    }

    public static double[] negate(double[] values) {
        double[] result = new double[values.length];
        for (int i = 0; i < values.length; i++) {
            result[i] = -values[i];
        }
        return result;
    }

    public static int[] negate(int[] values) {
        int[] result = new int[values.length];
        for (int i = 0; i < values.length; i++) {
            result[i] = -values[i];
        }
        return result;
    }

    public static void sleep(double time, MidnightClock.Resolution resolution) {
        try {
            Thread.sleep((long) ((time * resolution.multiplier) / MidnightClock.Resolution.MILLISECONDS.multiplier));
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}