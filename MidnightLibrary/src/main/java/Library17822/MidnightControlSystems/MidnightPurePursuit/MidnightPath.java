package Library17822.MidnightControlSystems.MidnightPurePursuit;

import java.util.Arrays;
import java.util.List;

import Library17822.MinightResources.MasqMath.MidnightPoint;
import Library17822.MinightResources.MasqMath.MidnightVector;
import Library17822.MidnightWrappers.MidnightDashBoard;

/**
 * Created by Archishmaan Peyyety on 8/11/18.
 * Project: MasqLib
 */

public class MidnightPath {
    private double lookAheadDistance = 5;
    private List<MidnightPoint> wayPoints;
    private MidnightVector end;
    private MidnightVector carrot;
    public MidnightPath(double lookAheadDistance, List<MidnightPoint> wayPoints) {
        this.lookAheadDistance = lookAheadDistance;
        this.wayPoints = wayPoints;
        end = new MidnightVector(wayPoints.get(1).getX() - wayPoints.get(0).getX(),
                wayPoints.get(1).getY() - wayPoints.get(0).getY());
    }
    public MidnightPath(double lookAheadDistance, MidnightPoint... wayPoints) {
        this.lookAheadDistance = lookAheadDistance;
        this.wayPoints = Arrays.asList(wayPoints);
        end = new MidnightVector(wayPoints[1].getX() - wayPoints[0].getX(),
                wayPoints[1].getY() - wayPoints[0].getY());
    }
    public void updateSystem (MidnightPoint robot) {
        MidnightVector robotVector = robot.toVector();
        MidnightVector unitVector = end.unitVector();
        //carrot = unitVector.multiply(robotVector.dotProduct(end) + lookAheadDistance);
    }
    public double getOrientationError(double angle, MidnightPoint robot) {
        MidnightVector directionVector = new MidnightVector(Math.sin(Math.toRadians(angle)), Math.cos(Math.toRadians(angle)), "direction");
        MidnightVector robotCarrot = new MidnightVector(carrot.getX() - robot.getX(),  carrot.getY() - robot.getY(), "robot");
        MidnightDashBoard.getDash().create(directionVector);
        MidnightDashBoard.getDash().create(robotCarrot);
        return Math.toDegrees(Math.acos((directionVector.dotProduct(robotCarrot)) / robotCarrot.getMagnitude()));
    }
    public void updatePath(MidnightPoint start, MidnightPoint end) {

    }
    public List<MidnightPoint> getWayPoints() {
        return wayPoints;
    }

    public double[] getQuad(double a, double b, double c) {
        double rootOne, rootTwo;
        rootOne = -(b + Math.sqrt(Math.pow(b, 2) - (4 * a * c))) / (2 * a);
        rootTwo = -(b - Math.sqrt(Math.pow(b, 2) - (4 * a * c))) / (2 * a);
        return new double[] {rootOne, rootTwo};
    }
    public MidnightVector getGoalPoint () {
        return carrot;
    }
}
