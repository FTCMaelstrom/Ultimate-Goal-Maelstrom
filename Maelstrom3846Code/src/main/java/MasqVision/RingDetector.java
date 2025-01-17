package MasqVision;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

import java.util.List;

import MidnightLibrary.MidnightMath.MidnightVector;

import static java.lang.Math.abs;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;
import static org.opencv.core.Core.extractChannel;
import static org.opencv.core.Core.mean;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2YCrCb;
import static org.opencv.imgproc.Imgproc.cvtColor;

/**
 * Created by Keval Kataria on 6/1/2020
 */

/*
 * Modified 4/20/21 9:06 PM by Amogh Mehta
 */

public class RingDetector extends MasqCVDetector {
    private final MasqCVColorFilter lumaFilter = new LumaFilter(100);
    double top, control, bottom;
    double prevTime = 0;
    private final boolean init = true;

    private double ratio = 1;
    private double x0 = 0;
    private double y0 = 0;

    @Override
    public Mat processFrame(Mat input) {
        double time = System.nanoTime();

        if (time - prevTime > 1e10) {
            if (init) {
                prevTime = time;
                workingMat = input.clone();
                displayMat = input.clone();

                cvtColor(workingMat, workingMat, COLOR_RGB2YCrCb);
                extractChannel(workingMat, workingMat, 1);

                Rect topRect = new Rect(tl, new Point(br.x, tl.y + (br.y - tl.y) * 3.0 / 4));
                Rect bottomRect = new Rect(new Point(tl.x, topRect.br().y), br);
                Rect controlRect = new Rect(new Point(tl.x, br.y), new Point(br.x, br.y + topRect.height + bottomRect.height));
                control = mean(workingMat.clone().submat(controlRect)).val[0];
                top = mean(workingMat.clone().submat(topRect)).val[0];
                bottom = mean(workingMat.clone().submat(bottomRect)).val[0];

                workingMat.release();

                drawRect(controlRect, new Scalar(255, 0, 0), false);
                drawRect(topRect, new Scalar(0, 0, 255), false);
                drawRect(bottomRect, new Scalar(0, 255, 0), false);
            } else {
                input.submat(new Rect(tl, br));
                workingMat = input.clone();
                displayMat = input.clone();

                List<MatOfPoint> contoursBright = findContours(lumaFilter, workingMat.clone());
                List<Rect> rectsBright = contoursToRects(contoursBright);
                List<List<Rect>> listsOfBrightBlobs = groupIntoBlobs(rectsBright, 10);
                List<Rect> rings = chooseRects(listsOfBrightBlobs);
                Rect bestRect = rings.get(0);
                Rect second = rings.get(1);

                drawContours(contoursBright, new Scalar(80, 80, 80));

                found = bestRect.area() > minimumArea;
                found2 = second.area() > minimumArea;

                workingMat.release();

                if (found) {
                    drawRect(bestRect, new Scalar(0, 255, 0), false);
                    drawCenterPoint(getCenterPoint(bestRect), new Scalar(0, 255, 0));
                    foundRect = bestRect;
                }
                if (found2) {
                    drawRect(second, new Scalar(0, 255, 0), false);
                    drawCenterPoint(getCenterPoint(second), new Scalar(0, 255, 0));
                    secondRect = second;
                }
            }
        }

        return displayMat;
    }

    public double getTop() {
        return top;
    }

    public double getBottom() {
        return bottom;
    }

    public double getControl() {
        return control;
    }

    /*
    public void switchDetection() {
        init = false;
    }
    */

    public TargetZone findZone() {
        if (abs(getTop() - getBottom()) > 10) return TargetZone.B;
        else if (abs(((getTop() + getBottom()) / 2 - getControl())) > 10) return TargetZone.C;
        else return TargetZone.A;
    }

    public MidnightVector[] findRings() {
        MidnightVector ring1 = null, ring2 = null;
        if (isFound() && isFound2()) {
            ring1 = new MidnightVector("Ring1", getCenterPoint(getFoundRect()).x - 480 + x0,
                    sqrt(pow(getFoundRect().height * ratio, 2) - pow(getCenterPoint(getFoundRect()).x, 2)) + y0);
            ring2 = new MidnightVector("Ring 2", getCenterPoint(getSecondRect()).x - 480 + x0,
                    sqrt(pow(getSecondRect().height * ratio, 2) - pow(getCenterPoint(getSecondRect()).x, 2)) + y0);
        } else if (isFound())
            ring2 = new MidnightVector("Ring 2", getCenterPoint(getSecondRect()).x - 480 + x0,
                    sqrt(pow(getSecondRect().height * ratio, 2) - pow(getCenterPoint(getSecondRect()).x, 2)) + y0);
        return new MidnightVector[]{ring1, ring2};
    }

    public void setRatio(double ratio) {
        this.ratio = ratio;
    }

    public void setDistances(double x0, double y0) {
        this.x0 = x0;
        this.y0 = y0;
    }

    public enum TargetZone {A, B, C}
}