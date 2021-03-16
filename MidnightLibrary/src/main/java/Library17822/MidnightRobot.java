package Library17822;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import Library17822.MidnightControlSystems.MidnightPID.MidnightPIDController;
import Library17822.MidnightControlSystems.MidnightPurePursuit.MidnightWayPoint;
import Library17822.MidnightControlSystems.MidnightPurePursuit.MidnightWayPointLegacy;
import Library17822.MidnightDriveTrains.MidnightMechanumDriveTrain;
import Library17822.MidnightSensors.MidnightClock;
import Library17822.MidnightWrappers.MidnightController;
import Library17822.MidnightWrappers.MidnightDashBoard;
import Library17822.MidnightWrappers.MidnightPredicate;
import Library17822.MinightResources.MasqMath.MidnightPoint;
import Library17822.MinightResources.MasqMath.MidnightVector;
import Library17822.MinightResources.MidnightHelpers.Direction;
import Library17822.MinightResources.MidnightUtils;

import static Library17822.MidnightControlSystems.MidnightPurePursuit.MidnightWayPoint.PointMode.MECH;
import static Library17822.MidnightControlSystems.MidnightPurePursuit.MidnightWayPoint.PointMode.SWITCH;
import static Library17822.MinightResources.MidnightUtils.DEFAULT_SLEEP_TIME;
import static Library17822.MinightResources.MidnightUtils.DEFAULT_SPEED_MULTIPLIER;
import static Library17822.MinightResources.MidnightUtils.DEFAULT_TIMEOUT;
import static Library17822.MinightResources.MidnightUtils.DEFAULT_TURN_MULTIPLIER;
import static Library17822.MinightResources.MidnightUtils.angleController;
import static Library17822.MinightResources.MidnightUtils.driveController;
import static Library17822.MinightResources.MidnightUtils.getLinearOpMode;
import static Library17822.MinightResources.MidnightUtils.max;
import static Library17822.MinightResources.MidnightUtils.scaleNumber;
import static Library17822.MinightResources.MidnightUtils.turnController;
import static Library17822.MinightResources.MidnightUtils.velocityAutoController;
import static Library17822.MinightResources.MidnightUtils.velocityTeleController;
import static Library17822.MinightResources.MidnightUtils.xyAngleController;
import static Library17822.MinightResources.MidnightUtils.xySpeedController;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.hypot;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;


/**
 * MidnightRobot--> Contains all hardware and methods to runLinearOpMode the robot.
TODO:
    Unit Tests for all major functions
    State Machine support
 */
public abstract class MidnightRobot {
    public abstract void mapHardware(HardwareMap hardwareMap);
    public abstract void init(HardwareMap hardwareMap) throws InterruptedException;

    public MidnightMechanumDriveTrain driveTrain;
    public MidnightPositionTracker tracker;
    public MidnightDashBoard dash;
    private MidnightClock timeoutClock = new MidnightClock();

    public static boolean opModeIsActive() {return MidnightUtils.opModeIsActive();}

    public void strafe(double distance, double angle, double timeout, double speed) {
        MidnightClock timeoutTimer = new MidnightClock();
        driveTrain.resetEncoders();
        double targetClicks = (int)(distance * driveTrain.getEncoder().getClicksPerInch());
        double clicksRemaining;
        double power, angularError, targetAngle = tracker.getHeading(), powerAdjustment;
        do {
            clicksRemaining = (int) (targetClicks - Math.abs(driveTrain.getCurrentPositionPositive()));
            power = driveController.getOutput(clicksRemaining) * speed;
            power = Range.clip(power, -1.0, +1.0);
            angularError = MidnightUtils.adjustAngle(targetAngle - tracker.getHeading());
            powerAdjustment = angleController.getOutput(MidnightUtils.adjustAngle(angularError));
            powerAdjustment = Range.clip(powerAdjustment, -1.0, +1.0);
            driveTrain.setVelocityMECH(angle, power, tracker.getHeading(), powerAdjustment);
            dash.create("ERROR: ", clicksRemaining);
            dash.create("HEADING: ", tracker.getHeading());
            dash.update();
        } while (opModeIsActive() && !timeoutTimer.elapsedTime(timeout, MidnightClock.Resolution.SECONDS) && (Math.abs(angularError) > 5 || clicksRemaining/targetClicks > 0.01));
        driveTrain.setVelocity(0);
        MidnightUtils.sleep();
    }
    public void strafe(double distance, double angle, double timeout) {
        strafe(distance, angle, timeout, 0.7);
    }
    public void strafe (double distance, double angle) {
        strafe(distance, angle, 1);
    }

    public void drive(double distance, double speed, Direction direction, double timeout, double sleepTime) {
        MidnightClock timeoutTimer = new MidnightClock();
        driveTrain.resetEncoders();
        double targetAngle = tracker.getHeading();
        double targetClicks = (int)(distance * driveTrain.getEncoder().getClicksPerInch());
        double clicksRemaining;
        double angularError, powerAdjustment, power, leftPower, rightPower, maxPower;
        do {
            clicksRemaining = (int) (targetClicks - Math.abs(driveTrain.getCurrentPosition()));
            power = driveController.getOutput(clicksRemaining) * speed;
            power = Range.clip(power, -1.0, +1.0);
            angularError = MidnightUtils.adjustAngle(targetAngle - tracker.getHeading());
            powerAdjustment = angleController.getOutput(MidnightUtils.adjustAngle(angularError));
            powerAdjustment = Range.clip(powerAdjustment, -1.0, +1.0);
            leftPower = (direction.value * power) - powerAdjustment;
            rightPower = (direction.value * power) + powerAdjustment;
            maxPower = max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxPower > 1.0) {
                leftPower /= maxPower;
                rightPower /= maxPower;
            }
            driveTrain.setVelocity(leftPower, rightPower);
            dash.create("LEFT POWER: ", leftPower);
            dash.create("RIGHT POWER: ", rightPower);
            dash.create("ERROR: ", clicksRemaining);
            dash.create("HEADING: ", tracker.getHeading());
            dash.update();
        } while (opModeIsActive() && !timeoutTimer.elapsedTime(timeout, MidnightClock.Resolution.SECONDS) && (Math.abs(angularError) > 5 || clicksRemaining/targetClicks > 0.01));
        driveTrain.setVelocity(0);
        MidnightUtils.sleep(sleepTime, MidnightClock.Resolution.SECONDS);
    }
    public void drive(double distance, double speed, Direction strafe, double timeout) {
        drive(distance, speed, strafe, timeout, MidnightUtils.DEFAULT_SLEEP_TIME);
    }
    public void drive(double distance, double speed, Direction strafe) {
        drive(distance, speed, strafe, MidnightUtils.DEFAULT_TIMEOUT);
    }
    public void drive(double distance, Direction direction, double timeout) {
        drive(distance, 1, direction, timeout);
    }
    public void drive(double distance, double speed){drive(distance, speed, Direction.FORWARD);}
    public void drive(double distance, Direction direction) {drive(distance, 0.5, direction);}
    public void drive(double distance) {drive(distance, 0.5);}

    public void driveAbsoluteAngle(double distance, int angle, double speed, Direction direction, double timeout, double sleepTime) {
        MidnightClock timeoutTimer = new MidnightClock();
        driveTrain.resetEncoders();
        double targetClicks = (int)(distance * driveTrain.getEncoder().getClicksPerInch());
        double clicksRemaining;
        double angularError, powerAdjustment, power, leftPower, rightPower, maxPower;
        do {
            clicksRemaining = (int) (targetClicks - Math.abs(driveTrain.getCurrentPosition()));
            power = driveController.getOutput(clicksRemaining) * speed;
            angularError = MidnightUtils.adjustAngle((double)angle - tracker.getHeading());
            powerAdjustment = angleController.getOutput(angularError);
            leftPower = power - powerAdjustment;
            rightPower = power + powerAdjustment;
            leftPower*=direction.value;
            rightPower*=direction.value;
            maxPower = max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxPower > 1.0) {
                leftPower /= maxPower;
                rightPower /= maxPower;
            }
            driveTrain.setVelocity(leftPower, rightPower);
            //serializer.writeData(new Object[]{clicksRemaining, power, angularError, angularIntegral, angularDerivative, leftPower, rightPower, powerAdjustment});
            dash.create("LEFT POWER: ", leftPower);
            dash.create("RIGHT POWER: ", rightPower);
            dash.create("ERROR: ", clicksRemaining);
            dash.update();
        } while (opModeIsActive() && !timeoutTimer.elapsedTime(timeout, MidnightClock.Resolution.SECONDS) && ((clicksRemaining / targetClicks) > 0.01));
        //serializer.close();
        driveTrain.setVelocity(0);
        MidnightUtils.sleep(sleepTime, MidnightClock.Resolution.SECONDS);
    }
    public void driveAbsoluteAngle(double distance, int angle, double speed, Direction strafe, double timeout) {
        driveAbsoluteAngle(distance, angle, speed, strafe, timeout, MidnightUtils.DEFAULT_SLEEP_TIME);
    }
    public void driveAbsoluteAngle(double distance, int angle, double speed, Direction strafe) {
        driveAbsoluteAngle(distance, angle, speed, strafe, MidnightUtils.DEFAULT_TIMEOUT);
    }
    public void driveAbsoluteAngle(double distance, int angle, double speed){
        driveAbsoluteAngle(distance, angle, speed, Direction.FORWARD);
    }
    public void driveAbsoluteAngle(double distance, int angle) {
        driveAbsoluteAngle(distance, angle, 0.5);
    }

    public void turnRelative(double angle, Direction direction, double timeout, double sleepTime, double kp, double ki, double kd, boolean left, boolean right) {
        double targetAngle = MidnightUtils.adjustAngle(tracker.getHeading()) + (direction.value * angle);
        double acceptableError = .5;
        double error = MidnightUtils.adjustAngle(targetAngle - tracker.getHeading());
        double power;
        double leftPower = 0, rightPower = 0;
        turnController.setConstants(kp, ki, kd);
        timeoutClock.reset();
        while (opModeIsActive() && (MidnightUtils.adjustAngle(Math.abs(error)) > acceptableError)
                && !timeoutClock.elapsedTime(timeout, MidnightClock.Resolution.SECONDS)) {
            error = MidnightUtils.adjustAngle(targetAngle - tracker.getHeading());
            power = turnController.getOutput(error);
            if (Math.abs(power) >= 1) power /= Math.abs(power);
            if (left) leftPower = power;
            if (right) rightPower = -power;
            driveTrain.setVelocity(leftPower, rightPower);
            dash.create("TargetAngle", targetAngle);
            dash.create("Heading", tracker.getHeading());
            dash.create("AngleLeftToCover", error);
            dash.create("Power: ", power);
            dash.create("Raw Power: ", driveTrain.getPower());
            dash.update();
        }
        driveTrain.setVelocity(0,0);
        MidnightUtils.sleep(sleepTime, MidnightClock.Resolution.SECONDS);
    }
    public void turnRelative(double angle, Direction direction, double timeout, double sleepTime, double kp, double ki) {
        turnRelative(angle, direction, timeout, sleepTime, kp, ki, turnController.getConstants()[2], true, true);
    }
    public void turnRelative(double angle, Direction direction, double timeout, double sleepTime, double kp) {
        turnRelative(angle, direction, timeout, sleepTime, kp, turnController.getConstants()[1]);
    }
    public void turnRelative(double angle, Direction direction, double timeout, double sleepTime) {
        turnRelative(angle, direction, timeout, sleepTime, turnController.getConstants()[0]);
    }
    public void turnRelative(double angle, Direction direction, double timeout) {
        turnRelative(angle, direction, timeout, MidnightUtils.DEFAULT_SLEEP_TIME);
    }
    public void turnRelative(double angle, Direction direction)  {
        turnRelative(angle, direction, MidnightUtils.DEFAULT_TIMEOUT);
    }
    public void turnRelative(double angle, Direction direction, boolean left, boolean right)  {
        turnRelative(angle, direction, MidnightUtils.DEFAULT_TIMEOUT, MidnightUtils.DEFAULT_SLEEP_TIME,
                turnController.getConstants()[0], turnController.getConstants()[1], turnController.getConstants()[2], left, right);
    }

    public void turnAbsolute(double angle,  double timeout, double sleepTime, double kp, double ki, double kd) {
        //double targetAngle = MidnightUtils.adjustAngle(angle);
        double acceptableError = 2;
        double error = MidnightUtils.adjustAngle(angle - tracker.getHeading());
        double power;
        turnController.setConstants(kp, ki, kd);
        timeoutClock.reset();
        while (opModeIsActive() && (MidnightUtils.adjustAngle(Math.abs(error)) > acceptableError)
                && !timeoutClock.elapsedTime(timeout, MidnightClock.Resolution.SECONDS)) {
            error = MidnightUtils.adjustAngle(angle - tracker.getHeading());
            power = turnController.getOutput(error);
            if (Math.abs(power) >= 1) power /= Math.abs(power);
            driveTrain.setVelocity(-power, power);
            dash.create("KP: ", kp);
            dash.create("RIGHT POWER: " ,power);
            dash.create("TargetAngle", angle);
            dash.create("Heading", tracker.getHeading());
            dash.create("AngleLeftToCover", error);
            dash.update();
        }
        driveTrain.setVelocity(0,0);
        MidnightUtils.sleep(sleepTime, MidnightClock.Resolution.SECONDS);
    }
    public void turnAbsolute(double angle, double timeout, double sleepTime,  double kp, double ki) {
        turnAbsolute(angle, timeout, sleepTime,  kp, ki, turnController.getKd());
    }
    public void turnAbsolute(double angle, double timeout, double sleepTime, double kp) {
        turnAbsolute(angle, timeout, sleepTime, kp, turnController.getKi());
    }
    public void turnAbsolute(double angle,  double timeout, double sleepTime) {
        turnAbsolute(angle, timeout, sleepTime,turnController.getKp());
    }
    public void turnAbsolute(double angle, double timeout)  {
        turnAbsolute(angle, timeout, DEFAULT_SLEEP_TIME);
    }
    public void turnAbsolute(double angle) {
        turnAbsolute(angle, DEFAULT_TIMEOUT);
    }

    public void stop(MidnightPredicate stopCondition, double angle, double speed, Direction direction, double timeout) {
        MidnightClock timeoutTimer = new MidnightClock();
        driveTrain.resetEncoders();
        double angularError, powerAdjustment, power, leftPower, rightPower, maxPower;
        do {
            power = direction.value * speed;
            power = Range.clip(power, -1.0, +1.0);
            angularError = MidnightUtils.adjustAngle(angle - tracker.getHeading());
            powerAdjustment = angleController.getOutput(angularError);
            powerAdjustment = Range.clip(powerAdjustment, -1.0, +1.0);
            powerAdjustment *= direction.value;
            leftPower = power - powerAdjustment;
            rightPower = power + powerAdjustment;
            maxPower = max(Math.abs(leftPower), Math.abs(rightPower));
            if (maxPower > 1.0) {
                leftPower /= maxPower;
                rightPower /= maxPower;
            }
            driveTrain.setVelocity(leftPower, rightPower);
            dash.create("LEFT POWER: ",leftPower);
            dash.create("RIGHT POWER: ",rightPower);
            dash.create("Angle Error", angularError);
            dash.update();
        } while (opModeIsActive() && !timeoutTimer.elapsedTime(timeout, MidnightClock.Resolution.SECONDS) && stopCondition.run());
        driveTrain.setVelocity(0);
    }
    public void stop(MidnightPredicate stopCondition, double angle, double speed, Direction direction) {
        stop(stopCondition, angle, speed, direction, MidnightUtils.DEFAULT_TIMEOUT);
    }
    public void stop(MidnightPredicate sensor, double angle, double power) {
        stop(sensor, angle, power, Direction.FORWARD);
    }
    public void stop(MidnightPredicate stopCondition, double angle) {
        stop(stopCondition, angle, 0.5);
    }
    public void stop(MidnightPredicate sensor){
        stop(sensor, tracker.getHeading());
    }
    public void stop(MidnightPredicate stopCondition, int timeout) {
        stop(stopCondition, tracker.getHeading(), 0.5, Direction.FORWARD, timeout);
    }

    public void xyPath(double timeout, MidnightWayPoint... points) {
        MidnightMechanumDriveTrain.angleCorrectionController.setKp(xyAngleController.getKp());
        MidnightMechanumDriveTrain.angleCorrectionController.setKi(xyAngleController.getKi());
        MidnightMechanumDriveTrain.angleCorrectionController.setKd(xyAngleController.getKd());
        List<MidnightWayPoint> pointsWithRobot = new ArrayList<>(Arrays.asList(points));
        pointsWithRobot.add(0, getCurrentWayPoint());
        MidnightPIDController travelAngleController = new MidnightPIDController(0.01, 0, 0);
        int index = 1;
        MidnightClock pointTimeout = new MidnightClock();
        timeoutClock.reset();
        while (!timeoutClock.elapsedTime(timeout, MidnightClock.Resolution.SECONDS) &&
                index < pointsWithRobot.size()) {
            double lookAheadDistance = pointsWithRobot.get(index).getLookAhead();
            travelAngleController.setKp(pointsWithRobot.get(index).getAngularCorrectionSpeed());
            MidnightMechanumDriveTrain.angleCorrectionController.setKp(pointsWithRobot.get(index).getAngularCorrectionSpeed());
            xySpeedController.setKp(pointsWithRobot.get(index).getDriveCorrectionSpeed());
            MidnightVector target = new MidnightVector(pointsWithRobot.get(index).getX(), pointsWithRobot.get(index).getY());
            MidnightVector current = new MidnightVector(tracker.getGlobalX(), tracker.getGlobalY());
            MidnightVector initial = new MidnightVector(pointsWithRobot.get(index - 1).getX(), pointsWithRobot.get(index - 1).getY());
            double speed = 1;
            pointTimeout.reset();
            while (!pointTimeout.elapsedTime(pointsWithRobot.get(index).getTimeout(), MidnightClock.Resolution.SECONDS) &&
                    !current.equal(pointsWithRobot.get(index).getTargetRadius(), target) && opModeIsActive() && speed > 0.1) {
                double heading = Math.toRadians(-tracker.getHeading());
                MidnightVector headingUnitVector = new MidnightVector(Math.sin(heading), Math.cos(heading));
                MidnightVector lookahead = MidnightUtils.getLookAhead(initial, current, target, lookAheadDistance);
                MidnightVector pathDisplacement = initial.displacement(target);
                boolean closerThanLookAhead = initial.displacement(lookahead).getMagnitude() > pathDisplacement.getMagnitude();
                boolean approachingFinalPos = index == pointsWithRobot.size() - 1;
                if (closerThanLookAhead) {
                    if (approachingFinalPos) lookahead = new MidnightVector(target.getX(), target.getY());
                    else break;
                }
                MidnightVector lookaheadDisplacement = current.displacement(lookahead);
                double pathAngle = MidnightUtils.adjustAngle(headingUnitVector.angleTan(lookaheadDisplacement));
                speed = xySpeedController.getOutput(current.displacement(target).getMagnitude());
                speed = scaleNumber(speed, pointsWithRobot.get(index).getMinVelocity(), pointsWithRobot.get(index).getMaxVelocity());
                double powerAdjustment = travelAngleController.getOutput(pathAngle);
                double leftPower = speed + powerAdjustment;
                double rightPower = speed - powerAdjustment;

                MidnightWayPoint.PointMode mode = pointsWithRobot.get(index).getSwitchMode();
                boolean mechMode = approachingFinalPos ||
                        (current.equal(pointsWithRobot.get(index).getModeSwitchRadius(), target) && mode == SWITCH) ||
                        mode == MECH;

                if (mechMode) {
                    pathAngle = 90 - Math.toDegrees(Math.atan2(lookaheadDisplacement.getY(), lookaheadDisplacement.getX()));
                    driveTrain.setVelocityMECH(
                            pathAngle + tracker.getHeading(), speed,
                            -pointsWithRobot.get(index).getH()
                    );
                }
                else driveTrain.setVelocity(leftPower, rightPower);

                dash.create(current.setName("Current MidnightSplinePoint"));
                dash.create("Heading: ", Math.toDegrees(heading));
                dash.create("Angular Correction Speed: ", MidnightMechanumDriveTrain.angleCorrectionController.getKp());
                dash.create("Drive Correction Speed: ", pointsWithRobot.get(index).getDriveCorrectionSpeed());
                dash.update();
                current = new MidnightVector(tracker.getGlobalX(), tracker.getGlobalY());
            }
            if (pointsWithRobot.get(index).getOnComplete() != null) pointsWithRobot.get(index).getOnComplete().run();
            index++;
        }
        driveTrain.setVelocity(0);
    }
    public void xyPathV2(double timeout, MidnightWayPoint... points) {
        MidnightMechanumDriveTrain.angleCorrectionController.setKp(xyAngleController.getKp());
        MidnightMechanumDriveTrain.angleCorrectionController.setKi(xyAngleController.getKi());
        MidnightMechanumDriveTrain.angleCorrectionController.setKd(xyAngleController.getKd());
        List<MidnightWayPoint> pointsWithRobot = new ArrayList<>(Arrays.asList(points));
        pointsWithRobot.add(0, getCurrentWayPoint());
        MidnightPIDController travelAngleController = new MidnightPIDController(0.01, 0, 0);
        int index = 1;
        MidnightClock pointTimeout = new MidnightClock();
        timeoutClock.reset();
        while (!timeoutClock.elapsedTime(timeout, MidnightClock.Resolution.SECONDS) &&
                index < pointsWithRobot.size()) {
            double lookAheadDistance = pointsWithRobot.get(index).getLookAhead();
            travelAngleController.setKp(pointsWithRobot.get(index).getAngularCorrectionSpeed());
            MidnightMechanumDriveTrain.angleCorrectionController.setKp(pointsWithRobot.get(index).getAngularCorrectionSpeed());
            MidnightVector target = new MidnightVector(pointsWithRobot.get(index).getX(), pointsWithRobot.get(index).getY());
            MidnightVector current = new MidnightVector(tracker.getGlobalX(), tracker.getGlobalY());
            MidnightVector initial = new MidnightVector(pointsWithRobot.get(index - 1).getX(), pointsWithRobot.get(index - 1).getY());
            double speed = 1;
            pointTimeout.reset();
            while (!pointTimeout.elapsedTime(pointsWithRobot.get(index).getTimeout(), MidnightClock.Resolution.SECONDS) &&
                    !current.equal(pointsWithRobot.get(index).getTargetRadius(), target) && opModeIsActive() && speed > 0.1) {
                double heading = Math.toRadians(-tracker.getHeading());
                // Y and X components are reversed because the axis are switched for the robot and a cartesian coordinate plane, where 0 degrees is east.
                // This robot should have 0 degrees at north.
                MidnightVector headingUnitVector = new MidnightVector(Math.sin(heading), Math.cos(heading));
                MidnightVector lookahead = MidnightUtils.getLookAhead(initial, current, target, lookAheadDistance);
                MidnightVector pathDisplacement = initial.displacement(target);
                boolean closerThanLookAhead = initial.displacement(lookahead).getMagnitude() > pathDisplacement.getMagnitude();
                boolean approachingFinalPos = index == pointsWithRobot.size() - 1;
                if (closerThanLookAhead) {
                    if (approachingFinalPos) lookahead = new MidnightVector(target.getX(), target.getY());
                    else break;
                }
                MidnightVector lookaheadDisplacement = current.displacement(lookahead);
                double pathAngle = MidnightUtils.adjustAngle(headingUnitVector.angleTan(lookaheadDisplacement));
                speed = xySpeedController.getOutput(current.displacement(target).getMagnitude());
                double maxVelocity = pointsWithRobot.get(index).getMaxVelocity();
                speed = scaleNumber(speed, pointsWithRobot.get(index).getMinVelocity(), maxVelocity);
                double powerAdjustment = travelAngleController.getOutput(pathAngle);


                MidnightWayPoint.PointMode mode = pointsWithRobot.get(index).getSwitchMode();
                boolean mechMode = approachingFinalPos ||
                        (current.equal(pointsWithRobot.get(index).getModeSwitchRadius(), target) && mode == SWITCH) ||
                        mode == MECH;

                if (mechMode) {
                    pathAngle = 90 - Math.toDegrees(Math.atan2(lookaheadDisplacement.getY(), lookaheadDisplacement.getX()));
                    driveTrain.setVelocityMECH(
                            pathAngle + tracker.getHeading(), speed,
                            -pointsWithRobot.get(index).getH()
                    );
                }
                else {
                    double pathAngleScaled = 1 - (Math.abs(pathAngle) / 180);
                    speed = speed * pathAngleScaled * pointsWithRobot.get(index).getSpeedBias();
                    double leftPower = speed + powerAdjustment;
                    double rightPower = speed - powerAdjustment;
                    driveTrain.setVelocity(leftPower, rightPower);
                }

                dash.create("pathAngle: ", pathAngle);

                current = new MidnightVector(tracker.getGlobalX(), tracker.getGlobalY());
                int i = 0;
                for (MidnightWayPoint point : pointsWithRobot) {
                    dash.create(point.setName("Index: " + i));
                    i++;
                }
                dash.update();
            }
            if (pointsWithRobot.get(index).getOnComplete() != null)
                pointsWithRobot.get(index).getOnComplete().run();
            index++;
        }
        driveTrain.setVelocity(0);
    }


    public void NFS(MidnightController c) {
        float move = -c.leftStickY();
        float turn = c.rightStickX() * 0.7f;
        double left = move + turn;
        double right = move - turn;
        double max = max(left, right);
        if(max > 1.0) {
            left /= max;
            right /= max;
        }
        driveTrain.setPower(left, right);
    }

    public void TANK(MidnightController c) {
        double left = -c.leftStickY();
        double right = -c.rightStickY();
        double leftRate = driveTrain.leftDrive.getVelocity();
        double rightRate = driveTrain.rightDrive.getVelocity();
        double maxRate = max(Math.abs(leftRate/left), Math.abs(rightRate/right));
        leftRate /= maxRate;
        rightRate /= maxRate;
        double leftError =  left - leftRate;
        double rightError = right - rightRate;
        driveTrain.rightDrive.setPower(right);
        driveTrain.leftDrive.setPower(left);
    }

    public void MECH(MidnightController c, Direction direction, boolean fieldCentric, double speedMultiplier, double turnMultiplier, boolean power) {
        int disable = 0;
        if (fieldCentric) disable = 1;

        double angle;

        double x = -c.leftStickY();
        double y = c.leftStickX();
        double xR = -c.rightStickX();

        angle = Math.atan2(y, x) + (Math.toRadians(tracker.getHeading()) * disable);
        double adjustedAngle = angle + Math.PI/4;

        double speedMagnitude = Math.hypot(x, y);

        double leftFront = (Math.sin(adjustedAngle) * speedMagnitude * speedMultiplier) - xR * turnMultiplier * direction.value;
        double leftBack = (Math.cos(adjustedAngle) * speedMagnitude * speedMultiplier) - xR  * turnMultiplier * direction.value;
        double rightFront = (Math.cos(adjustedAngle) * speedMagnitude * speedMultiplier) + xR * turnMultiplier * direction.value;
        double rightBack = (Math.sin(adjustedAngle) * speedMagnitude * speedMultiplier) + xR * turnMultiplier * direction.value;

        double max = max(Math.abs(leftFront), Math.abs(leftBack), Math.abs(rightFront), Math.abs(rightBack));
        if (max > 1) {
            leftFront /= Math.abs(max);
            leftBack /= Math.abs(max);
            rightFront /= Math.abs(max);
            rightBack /= Math.abs(max);
        }
        if (!power) {
            driveTrain.leftDrive.motor1.setVelocity(leftFront * direction.value);
            driveTrain.leftDrive.motor2.setVelocity(leftBack * direction.value);
            driveTrain.rightDrive.motor1.setVelocity(rightFront * direction.value);
            driveTrain.rightDrive.motor2.setVelocity(rightBack * direction.value);
        } else {
            driveTrain.leftDrive.motor1.setPower(leftFront * direction.value);
            driveTrain.leftDrive.motor2.setPower(leftBack * direction.value);
            driveTrain.rightDrive.motor1.setPower(rightFront * direction.value);
            driveTrain.rightDrive.motor2.setPower(rightBack * direction.value);
        }
    }
    public void MECH(MidnightController c, Direction direction) {
        MECH(c, direction, false, DEFAULT_SPEED_MULTIPLIER, DEFAULT_TURN_MULTIPLIER, false);
    }
    public void MECH(MidnightController c, boolean disabled) {
        MECH(c, Direction.FORWARD, disabled, DEFAULT_SPEED_MULTIPLIER, DEFAULT_TURN_MULTIPLIER, false);
    }
    public void MECH(MidnightController c, boolean fieldCentric, boolean power) {
        MECH(c, Direction.FORWARD, fieldCentric, DEFAULT_SPEED_MULTIPLIER, DEFAULT_TURN_MULTIPLIER, power);
    }
    public void MECH(MidnightController c) {
        MECH(c, Direction.FORWARD, false, DEFAULT_SPEED_MULTIPLIER, DEFAULT_TURN_MULTIPLIER, false);
    }
    public void MECH(MidnightController c, double speedMutliplier, double turnMultiplier) {
        MECH(c, Direction.FORWARD, false, speedMutliplier, turnMultiplier, false);
    }

    public void initializeTeleop(){
        driveTrain.setKp(velocityTeleController.getKp());
        driveTrain.setKi(velocityTeleController.getKi());
        driveTrain.setKd(velocityTeleController.getKd());
    }
    public void initializeAutonomous() {
        driveTrain.setKp(velocityAutoController.getKp());
        driveTrain.setKi(velocityAutoController.getKi());
        driveTrain.setKd(velocityAutoController.getKd());
    }
    public MidnightWayPoint getCurrentWayPoint() {
        return new MidnightWayPoint().setPoint(new MidnightPoint(tracker.getGlobalX(), tracker.getGlobalY(), tracker.getHeading())).setName("Inital WayPoint");
    }
    public MidnightWayPointLegacy getCurrentWayPointLegacy() {
        return new MidnightWayPointLegacy(new MidnightPoint(tracker.getGlobalX(), tracker.getGlobalY(), tracker.getHeading()));
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

}