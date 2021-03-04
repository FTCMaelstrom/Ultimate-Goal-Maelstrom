package Library17822.MidnightControlSystems.MidnightSplineGenerator;

import java.util.ArrayList;
import java.util.List;

import Jama.Matrix;

public class MidnightSplineGenerator {
    private static List<MidnightSplinePoint> knots = new ArrayList<>();
    private static double resolution = 0.01;

    public static void addKnot(MidnightSplinePoint p) {
        knots.add(p);
    }

    private static Matrix generateConstraintMatrix(List<Double> ns) {
        // Each point apart from the initial point, requires 4 constraints. N represents the number of constraints.
        int numPoints = ns.size();
        int n = (numPoints - 1) * 4;
        double matrix[][] = new double[n][n];
        if (numPoints == 1) return null;
        // Initial velocity = 0
        matrix[0][1] = 1;
        // Final velocity = 0
        matrix[1][n-1] = 3;
        matrix[1][n-2] = 2;
        matrix[1][n-3] = 1;

        // We filled equations 0 & 1 with initial and final velocity constraint.
        int equationIndex = 2;
        int pointIndex = 1;
        while (equationIndex < n) {
            // Start of segment is equal to initial point.
            int segmentIndexStart = ((pointIndex - 1) * 4);
            matrix[equationIndex][segmentIndexStart] = 1;
            equationIndex++;
            // End of segment is equal to destination.
            int segmentCoeffIndex = 0;
            while (segmentCoeffIndex < 4) {
                matrix[equationIndex][segmentCoeffIndex + segmentIndexStart] = 1;
                segmentCoeffIndex++;
            }
            equationIndex++;
            // Smooth velocities and accelerations if there is another point left to complete
            if (pointIndex < numPoints - 1) {
                // Velocity Constraint
                matrix[equationIndex][segmentIndexStart + 1] = 1;
                matrix[equationIndex][segmentIndexStart + 2] = 2;
                matrix[equationIndex][segmentIndexStart + 3] = 3;
                matrix[equationIndex][segmentIndexStart + 5] = -1;
                equationIndex++;

                matrix[equationIndex][segmentIndexStart + 2] = 2;
                matrix[equationIndex][segmentIndexStart + 3] = 6;
                matrix[equationIndex][segmentIndexStart + 6] = -2;
                equationIndex++;
            }

            pointIndex++;
        }
        return new Matrix(matrix);
    }

    private static Matrix generateSolutionsMatrix(List<Double> ns) {
        int numPoints = ns.size();
        int n = (numPoints - 1) * 4;
        double[][] matrix = new double[n][1];
        int pointIndex = 0, solutionIndex = 0;
        while (solutionIndex < n) {
            matrix[solutionIndex + 2][0] = ns.get(pointIndex);
            matrix[solutionIndex + 3][0] = ns.get(pointIndex + 1);
            solutionIndex += 4;
            pointIndex++;
        }
        return new Matrix(matrix);
    }

    private static List<MidnightCubicSegment> getSegments(List<Double> ns) {
        List<MidnightCubicSegment> segments = new ArrayList<>();
        Matrix constraints = generateConstraintMatrix(ns);
        Matrix solutions = generateSolutionsMatrix(ns);
        if (constraints == null) return null;
        Matrix coeffs = constraints.solve(solutions);
        int n = coeffs.getArray().length;
        int index = 0;
        while (index < n) {
            segments.add(new MidnightCubicSegment(coeffs.get(index + 3, 0), coeffs.get(index + 2, 0), coeffs.get(index + 1, 0), coeffs.get(index, 0)));
            index += 4;
        }
        return segments;
    }

    public static List<MidnightSplinePoint> generatePoints() {
        List<Double> xs = new ArrayList<>();
        List<Double> ys = new ArrayList<>();
        List<Double> hs = new ArrayList<>();
        for (MidnightSplinePoint p : knots) {
            xs.add(p.getX());
            ys.add(p.getY());
            hs.add(p.getH());
        }

        List<MidnightCubicSegment> xSegments = getSegments(xs);
        List<MidnightCubicSegment> ySegments = getSegments(ys);
        List<MidnightCubicSegment> hSegments = getSegments(hs);
        if (xSegments == null || ySegments == null || hSegments == null) return null;
        List<MidnightSplinePoint> midnightSplinePoints = new ArrayList<>();

        double t = 0;
        // segments = knots - 1
        int n = knots.size() - 1;
        int index = 0;
        while (index < n) {
            while (t < 1) {
                midnightSplinePoints.add(new MidnightSplinePoint(xSegments.get(index).compute(t), ySegments.get(index).compute(t), hSegments.get(index).compute(t)));
                t += resolution;
            }
            t = 0;
            index++;
        }
        return midnightSplinePoints;
    }

    private static void printMatrix(double[][] matrix) {
        for (double[] aMatrix : matrix) {
            for (double anAMatrix : aMatrix)
                System.out.print(anAMatrix + " ");
            System.out.println();
        }
    }

}
