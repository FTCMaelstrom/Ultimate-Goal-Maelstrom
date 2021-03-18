package MaelstromCV.UltimateGoalSpecific;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;

import MaelstromCV.CVDetector;
import MaelstromCV.MidnightCVFilters.MidnightCVColorFilter;
import MaelstromCV.MidnightCVFilters.YCbCrLuminanceFilter;
import MidnightLibrary.MidnightMath.MidnightVector;

import static java.lang.Math.abs;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;
import static org.opencv.core.Core.extractChannel;
import static org.opencv.core.Core.mean;
import static org.opencv.imgproc.Imgproc.cvtColor;
/**
 * Created by Amogh Mehta
 * Project: FtcRobotController_Ultimate-Goal_prod2
 * Last Modified: 3/17/21 6:01 PM
 * Last Updated: 3/17/21 10:47 PM
 **/
public class RingHunter extends CVDetector {
    double top;
    double control;
    double bottom;
    double previousTime = 0;

    private boolean init = true;
    private double ratio = 1;
    private double x0 = 0;
    private double y0 = 0;

    private final MidnightCVColorFilter midnightCVColorFilter = new YCbCrLuminanceFilter(100);

    @Override
    public Mat processFrame(Mat input) {
        double currentTime = System.nanoTime();

        if (currentTime - previousTime > 3e9) {
            if (init) {
                previousTime = currentTime;

                workingMat = input.clone();
                displayMat = input.clone();

                cvtColor(workingMat, workingMat, Imgproc.COLOR_RGB2YCrCb);
                extractChannel(workingMat, workingMat, 1);

                Rect topRectangle = new Rect(tl, new Point(br.x, tl.y + (br.y - tl.y) * 3.0 / 4.0));
                Rect bottomRectangle = new Rect(new Point(tl.x, topRectangle.br().y), br);
                Rect controlRectangle = new Rect(new Point(tl.x, br.y), new Point(br.x, br.y + topRectangle.height + bottomRectangle.height));

                control = mean(workingMat.clone().submat(controlRectangle)).val[0];
                top = mean(workingMat.clone().submat(topRectangle)).val[0];
                bottom = mean(workingMat.clone().submat(bottomRectangle)).val[0];

                workingMat.release();

                drawRectangle(controlRectangle, new Scalar(255, 0, 0), false); //Red rectangle for control
                drawRectangle(bottomRectangle, new Scalar(0, 255, 0), false); //Green rectangle for bottom
                drawRectangle(topRectangle, new Scalar(0, 0, 255), false); //Blue rectangle for top
            } else {
                input.submat(new Rect(tl, br));


                workingMat = input.clone();
                displayMat = input.clone();

                List<MatOfPoint> contoursThatAreBright = findContours(midnightCVColorFilter, workingMat.clone());

                List<Rect> rectsThatAreBright = contoursToRects(contoursThatAreBright);

                List<List<Rect>> listOfBrightGroups = groupIntoGroups(rectsThatAreBright, 10);

                Rect[] rings = chooseTwoRectangles(listOfBrightGroups);
                Rect bestRect = rings[0];
                Rect secondBestRect = rings[0];

                drawContours(contoursThatAreBright, new Scalar(80, 80, 80));//Gray contours around bright things

                found = bestRect.area() > minArea;
                found2 = secondBestRect.area() > minArea;
                workingMat.release();


                if (found) {
                    drawRectangle(bestRect, new Scalar(0, 255, 0), false);
                    drawCenterPoint(getCenterPoint(bestRect), new Scalar(0, 255, 0));//Green center point around Rect1
                    foundRect = bestRect;
                }

                if (found2) {
                    drawRectangle(secondBestRect, new Scalar(0, 255, 0), false);
                    drawCenterPoint(getCenterPoint(secondBestRect), new Scalar(0, 255, 0));//Green center point around Rect2
                    secondRect = secondBestRect;
                }
            }
        }
        return displayMat;
    }
        public double getControl() {
            return control;
        }

        public double getTop() {
            return top;
        }

        public double getBottom() {
            return bottom;
        }

        public void switchDetect() {
            init = false;
        }

        public enum TargetZone{
            A,B,C
        }
        public TargetZone findZone() {
            if (abs(getTop() - getBottom()) > 15) {
                return TargetZone.B;
            } else if (abs(((getTop() + getBottom()) / 2 - getControl())) > 10) {
                return TargetZone.C;
            } else {
                return TargetZone.A;
            }
        }

        public MidnightVector[] ringFinder() {
            MidnightVector ring1 = null;
            MidnightVector ring2 = null;

            if (isFound() && isFound2()) {
                ring1 = new MidnightVector(getCenterPoint(getFoundRect()).x - 480 + x0, sqrt(pow(getFoundRect().height * ratio, 2) - pow(getCenterPoint(getFoundRect()).x, 2)) + y0, "Ring1");
                ring2 = new MidnightVector(getCenterPoint(getSecondRect()).x - 480 + x0, sqrt(pow(getSecondRect().height * ratio, 2) - pow(getCenterPoint(getSecondRect()).x, 2)) + y0, "Ring2");
            } else if (isFound())
                ring2 = new MidnightVector(getCenterPoint(getSecondRect()).x - 480 + x0, sqrt(pow(getSecondRect().height * ratio, 2) - pow(getCenterPoint(getSecondRect()).x, 2)) + y0, "Ring2");
            return new MidnightVector[]{
                    ring1, ring2
            };
        }

        public void setRatio(double ratio) {
            this.ratio = ratio;
        }

        public void setDistances(double x0, double y0) {
            this.x0 = x0;
            this.y0 = y0;
        }

}
