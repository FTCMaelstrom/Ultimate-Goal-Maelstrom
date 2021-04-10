package MidnightLibrary;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.List;

import MidnightLibrary.MidnightMath.MidnightPIDController;
import MidnightLibrary.MidnightMath.MidnightPoint;
import MidnightLibrary.MidnightMath.MidnightVector;
import MidnightLibrary.MidnightMath.MidnightWayPoint;
import MidnightLibrary.MidnightDrivetrain.MidnightMechanumDriveTrain;
import MidnightLibrary.MidnightDrivetrain.MidnightPositionTracker;
import MidnightLibrary.MidnightResources.Direction;
import MidnightLibrary.MidnightResources.MidnightClock;
import MidnightLibrary.MidnightResources.MidnightDashBoard;

import static MidnightLibrary.MidnightMath.MidnightWayPoint.PointMode.MECH;
import static MidnightLibrary.MidnightMath.MidnightWayPoint.PointMode.SWITCH;
import static MidnightLibrary.MidnightResources.Direction.FORWARD;
import static MidnightLibrary.MidnightResources.MidnightClock.Resolution.SECONDS;
import static MidnightLibrary.MidnightResources.MidnightUtils.DEFAULT_SPEED_MULTIPLIER;
import static MidnightLibrary.MidnightResources.MidnightUtils.DEFAULT_TIMEOUT;
import static MidnightLibrary.MidnightResources.MidnightUtils.DEFAULT_TURN_MULTIPLIER;
import static MidnightLibrary.MidnightResources.MidnightUtils.adjustAngle;
import static MidnightLibrary.MidnightResources.MidnightUtils.angleController;
import static MidnightLibrary.MidnightResources.MidnightUtils.driveController;
import static MidnightLibrary.MidnightResources.MidnightUtils.getLinearOpMode;
import static MidnightLibrary.MidnightResources.MidnightUtils.getLookAhead;
import static MidnightLibrary.MidnightResources.MidnightUtils.max;
import static MidnightLibrary.MidnightResources.MidnightUtils.opModeIsActive;
import static MidnightLibrary.MidnightResources.MidnightUtils.scaleNumber;
import static MidnightLibrary.MidnightResources.MidnightUtils.turnController;
import static com.qualcomm.robotcore.util.Range.clip;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.hypot;
import static java.lang.Math.sin;
import static java.lang.Math.toDegrees;
import static java.lang.Math.toRadians;
import static java.util.Arrays.asList;


/**
 * MidnightRobot--> Contains all hardware and methods to runLinearOpMode the robot.
 * TODO:
 * Unit Tests for all major functions
 * State Machine support
 */
public abstract class MidnightRobot {
    public MidnightMechanumDriveTrain driveTrain;
    public MidnightPositionTracker tracker;
    protected MidnightDashBoard dash;
    private final MidnightClock timeoutClock = new MidnightClock();

    public abstract void mapHardware(HardwareMap hardwareMap);

    public abstract void init(HardwareMap hardwareMap, OpMode opmode);

    public void drive(double distance, Direction direction, double timeout) {
        double targetAngle = tracker.getHeading();
        double targetClicks = distance * driveTrain.getEncoder().getClicksPerInch();
        double clicksRemaining, angularError, powerAdjustment, power, leftPower, rightPower, maxPower;

        driveTrain.resetEncoders();
        timeoutClock.reset();

        do {
            clicksRemaining = targetClicks - abs(driveTrain.getCurrentPosition());
            power = driveController.getOutput(clicksRemaining);
            power = clip(power, -1, 1);
            angularError = adjustAngle(targetAngle - tracker.getHeading());
            powerAdjustment = angleController.getOutput(angularError);
            powerAdjustment = clip(powerAdjustment, -1, 1);
            leftPower = (direction.value * power) - powerAdjustment;
            rightPower = (direction.value * power) + powerAdjustment;

            maxPower = Math.max(abs(leftPower), abs(rightPower));
            if (maxPower > 1) {
                leftPower /= maxPower;
                rightPower /= maxPower;
            }

            tracker.updateSystem();
            driveTrain.setVelocity(leftPower, rightPower);

            dash.create("LEFT POWER: ", leftPower);
            dash.create("RIGHT POWER: ", rightPower);
            dash.create("ERROR: ", clicksRemaining);
            dash.create("HEADING: ", tracker.getHeading());
            dash.update();
        } while (opModeIsActive() && timeoutClock.hasNotPassed(timeout, SECONDS) && (abs(angularError) > 5 || clicksRemaining / targetClicks > 0.01));
        driveTrain.setVelocity(0);
    }

    public void drive(double distance, Direction direction) {
        drive(distance, direction, DEFAULT_TIMEOUT);
    }

    public void drive(double distance) {
        drive(distance, FORWARD);
    }

    public void turnRelative(double angle, Direction direction, double timeout) {
        double targetAngle = adjustAngle(tracker.getHeading()) + (direction.value * angle);
        double acceptableError = 0.5;
        double error, power;

        timeoutClock.reset();
        do {
            error = adjustAngle(targetAngle - tracker.getHeading());
            power = turnController.getOutput(error);
            if (abs(power) > 1) power /= abs(power);
            driveTrain.setVelocity(power, -power);

            dash.create("TargetAngle", targetAngle);
            dash.create("Heading", tracker.getHeading());
            dash.create("AngleLeftToCover", error);
            dash.create("Power: ", power);
            dash.create("Raw Power: ", driveTrain.getPower());
            dash.update();
        } while (opModeIsActive() && (adjustAngle(abs(error)) > acceptableError) && timeoutClock.hasNotPassed(timeout, SECONDS));
        driveTrain.setVelocity(0);
    }

    public void turnRelative(double angle, Direction direction) {
        turnRelative(angle, direction, DEFAULT_TIMEOUT);
    }

    public void turnAbsolute(double angle, double timeout, double acceptableError) {
        double error;
        double power;

        timeoutClock.reset();
        do {
            error = adjustAngle(angle - tracker.getHeading());
            power = clip(turnController.getOutput(error), -1, 1);

            driveTrain.setVelocity(power, -power);
            tracker.updateSystem();

            dash.create("KP: ", turnController.getKp());
            dash.create("Power: ", power);
            dash.create("TargetAngle: ", angle);
            dash.create("Heading: ", tracker.getHeading());
            dash.create("AngleLeftToCover: ", error);
            dash.update();
        } while (opModeIsActive() && (adjustAngle(abs(error)) > acceptableError) && timeoutClock.hasNotPassed(timeout, SECONDS));
        driveTrain.setVelocity(0);
    }

    public void turnAbsolute(double angle, double timeout) {
        turnAbsolute(angle, timeout, 1);
    }

    public void turnAbsolute(double angle) {
        turnAbsolute(angle, DEFAULT_TIMEOUT);
    }

    public void stopWhen(boolean stopCondition, double angle, double speed, Direction direction, double timeout) {
        double angularError, powerAdjustment, power, leftPower, rightPower, maxPower;

        timeoutClock.reset();
        do {
            power = direction.value * speed;
            power = clip(power, -1.0, +1.0);
            angularError = adjustAngle(angle - tracker.getHeading());
            powerAdjustment = angleController.getOutput(angularError);
            powerAdjustment = clip(powerAdjustment, -1.0, +1.0);
            leftPower = power + powerAdjustment;
            rightPower = power - powerAdjustment;

            maxPower = Math.max(abs(leftPower), abs(rightPower));
            if (maxPower > 1) {
                leftPower /= maxPower;
                rightPower /= maxPower;
            }

            driveTrain.setVelocity(leftPower, rightPower);
            tracker.updateSystem();

            dash.create("LEFT POWER: ", leftPower);
            dash.create("RIGHT POWER: ", rightPower);
            dash.create("Angle Error", angularError);
            dash.update();
        } while (opModeIsActive() && timeoutClock.hasNotPassed(timeout, SECONDS) && !stopCondition);
        driveTrain.setVelocity(0);
    }

    public void stopWhen(boolean stopCondition, double angle, double speed, Direction direction) {
        stopWhen(stopCondition, angle, speed, direction, DEFAULT_TIMEOUT);
    }

    public void stopWhen(boolean sensor, double angle, double power) {
        stopWhen(sensor, angle, power, FORWARD);
    }

    public void stopWhen(boolean stopCondition, double angle) {
        stopWhen(stopCondition, angle, 0.5);
    }

    public void stopWhen(boolean sensor) {
        stopWhen(sensor, tracker.getHeading());
    }

    public void stopWhen(boolean stopCondition, int timeout) {
        stopWhen(stopCondition, tracker.getHeading(), 0.5, FORWARD, timeout);
    }

    public void xyPath(double timeout, MidnightWayPoint... points) {
        List<MidnightWayPoint> pointsWithRobot = new ArrayList<>(asList(points));
        pointsWithRobot.add(0, getCurrentWayPoint());
        MidnightPIDController speedController = new MidnightPIDController();
        int index = 1;
        MidnightClock pointTimeout = new MidnightClock();
        timeoutClock.reset();
        while (timeoutClock.hasNotPassed(timeout, SECONDS) &&
                index < pointsWithRobot.size()) {
            double lookAheadDistance = pointsWithRobot.get(index).getLookAhead();
            angleController.setKp(pointsWithRobot.get(index).getAngularCorrectionSpeed());
            speedController.setKp(pointsWithRobot.get(index).getDriveCorrectionSpeed());
            MidnightWayPoint target = pointsWithRobot.get(index);
            MidnightVector current = new MidnightVector(tracker.getGlobalX(), tracker.getGlobalY());
            MidnightVector initial = new MidnightVector(pointsWithRobot.get(index - 1).getX(), pointsWithRobot.get(index - 1).getY());
            double speed = 1;
            double pathAngle;
            pointTimeout.reset();
            while (pointTimeout.hasNotPassed(pointsWithRobot.get(index).getTimeout(), SECONDS) &&
                    !current.equal(pointsWithRobot.get(index).getTargetRadius(), target.getPoint()) && opModeIsActive() && speed > 0.1) {
                double heading = toRadians(tracker.getHeading());
                MidnightVector headingUnitVector = new MidnightVector(sin(heading), cos(heading));
                MidnightVector lookahead = getLookAhead(initial, current, target.getPoint(), lookAheadDistance);
                MidnightVector pathDisplacement = initial.displacement(target.getPoint());
                boolean closerThanLookAhead = initial.displacement(lookahead).getMagnitude() > pathDisplacement.getMagnitude();
                boolean approachingFinalPos = index == pointsWithRobot.size() - 1;
                if (closerThanLookAhead) {
                    if (approachingFinalPos) lookahead = target.getPoint();
                    else break;
                }
                MidnightVector lookaheadDisplacement = current.displacement(lookahead);
                speed = speedController.getOutput(current.displacement(target.getPoint()).getMagnitude());
                speed = scaleNumber(speed, target.getMinVelocity(), target.getMaxVelocity());

                MidnightWayPoint.PointMode mode = target.getSwitchMode();
                boolean mechMode = (current.equal(target.getModeSwitchRadius(), target.getPoint()) && mode == SWITCH) ||
                        mode == MECH;

                if (mechMode) {
                    pathAngle = 90 - toDegrees(atan2(lookaheadDisplacement.getY(), lookaheadDisplacement.getX()));
                    driveTrain.setVelocityMECH(pathAngle - tracker.getHeading(), speed, target.getH());
                } else {
                    pathAngle = adjustAngle(headingUnitVector.angleTan(lookaheadDisplacement));
                    double powerAdjustment = angleController.getOutput(pathAngle);
                    double leftPower = speed + powerAdjustment;
                    double rightPower = speed - powerAdjustment;

                    int direction = 1;
                    if (abs(pathAngle) > 100) direction = -1;

                    driveTrain.setVelocity(direction * leftPower, direction * rightPower);
                }

                tracker.updateSystem();

                dash.create(tracker);
                dash.create("Distance Left", target.getPoint().displacement(current).getMagnitude());
                dash.create("Path Angle: ", pathAngle);
                dash.update();

                current = new MidnightVector(tracker.getGlobalX(), tracker.getGlobalY());
            }
            pointsWithRobot.get(index).getOnComplete().run();
            index++;
        }
        driveTrain.setVelocity(0);
    }

    public void xyPath(MidnightWayPoint... points) {
        double timeout = 0;
        for (MidnightWayPoint point : points) timeout += point.getTimeout();
        xyPath(timeout, points);
    }

    public void NFS(Gamepad c) {
        float move = -c.left_stick_y;
        float turn = c.right_stick_x * 0.7f;
        double left = move + turn;
        double right = move - turn;

        double max = Math.max(left, right);
        if (max > 1.0) {
            left /= max;
            right /= max;
        }
        driveTrain.setVelocity(left, right);
    }

    public void TANK(Gamepad c) {
        driveTrain.rightDrive.setVelocity(c.right_stick_y);
        driveTrain.leftDrive.setVelocity(c.left_stick_y);
    }


    public void MECH(Gamepad c, boolean fieldCentric, double speedMultiplier, double turnMultiplier) {
        int disable = 0;
        if (fieldCentric) disable = 1;

        double x = c.left_stick_x;
        double y = -c.left_stick_y;
        double xR = c.right_stick_x;


        double angle = atan2(x, y) + (toRadians(tracker.getHeading()) * disable);
        double adjustedAngle = angle + PI / 4;

        double speedMagnitude = hypot(x, y) * speedMultiplier;
        double turnMagnitude = xR * turnMultiplier;

        double leftFront = (sin(adjustedAngle) * speedMagnitude) + turnMagnitude;
        double leftBack = (cos(adjustedAngle) * speedMagnitude) + turnMagnitude;
        double rightFront = (cos(adjustedAngle) * speedMagnitude) - turnMagnitude;
        double rightBack = (sin(adjustedAngle) * speedMagnitude) - turnMagnitude;

        double max = max(abs(leftFront), abs(leftBack), abs(rightFront), abs(rightBack));
        if (max > 1) {
            leftFront /= abs(max);
            leftBack /= abs(max);
            rightFront /= abs(max);
            rightBack /= abs(max);
        }

        driveTrain.setVelocity(leftFront, leftBack, rightFront, rightBack);
    }

    public void MECH(Gamepad c, boolean fieldCentric) {
        MECH(c, fieldCentric, DEFAULT_SPEED_MULTIPLIER, DEFAULT_TURN_MULTIPLIER);
    }

    public void MECH(Gamepad c) {
        MECH(c, false);
    }

    public void MECH() {
        MECH(getLinearOpMode().getDefaultController());
    }

    public MidnightWayPoint getCurrentWayPoint() {
        return new MidnightWayPoint().setPoint(new MidnightPoint(tracker.getGlobalX(), tracker.getGlobalY(), tracker.getHeading())).setName("Inital WayPoint");
    }

    public enum OpMode {
        AUTO, TELEOP
    }

}