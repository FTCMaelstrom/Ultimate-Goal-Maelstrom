package MaelstromCV;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import MaelstromCV.MidnightCVFilters.MidnightCVColorFilter;
/**
 * Created by Amogh Mehta
 * Project: FtcRobotController_Ultimate-Goal_prod2
 * Last Modified: 3/16/21 4:10 PM
 * Last Updated: 3/16/21 4:11 PM
 **/
public class CVDetector extends OpenCvPipeline {
    protected int minArea=1, imageWidth=1280, imageHeight=960;
    protected boolean found=false, found2=false;
    protected Rect foundRect=new Rect(), secondRect=new Rect();
    protected Mat workingMat, displayMat;
    protected Point tl, br;

    public double distance(Point a, Point b) {
        return Math.sqrt(Math.pow(b.x-a.x,2)+Math.pow(b.y-a.y,2));
    }

    public Point getCenterPoint(Rect rect) {
        return new Point(rect.x+rect.width/2.0, rect.y+rect.height/2.0);
    }

    private Rect boundingRectangle(List<Rect> rects) {
        int minimumX=999,minimumY=999,maximumX=0,maximumY=0;
        for (Rect rectangles : rects) {
            minimumX = Math.min(rectangles.x, minimumX);
            minimumY = Math.min(rectangles.y, minimumY);
            maximumX = Math.max(rectangles.x+rectangles.width, maximumX);
        }
        return new Rect(minimumX, minimumY, maximumX-minimumX,maximumY-minimumY);
    }

    protected void drawRectangle (Rect rectangle, Scalar scalar, boolean fill) {
        if (fill) {
            Imgproc.rectangle(displayMat, rectangle.tl(), rectangle.br(), scalar, -1);
        } else {
            Imgproc.rectangle(displayMat, rectangle.tl(), rectangle.br(), scalar, 2);
        }
    }

    protected List <Rect> filterByBound(List<Rect> rects, Rect boundingRectangle) {
        List<Rect> rectsInsideBound = new ArrayList<>();
        for (Rect rect : rects) {
            if (boundingRectangle.contains(getCenterPoint(rect))) rectsInsideBound.add(rect);
        }
        return rectsInsideBound;
    }

    protected Rect[] chooseTwoRectangles (List<List<Rect>> listOfGroupsCTR) {
        Rect bestRectangle = new Rect();
        Rect secondBestRectangle = new Rect();

        try {
            bestRectangle = boundingRectangle(listOfGroupsCTR.get(0));
        } catch (Exception exception) {
            exception.printStackTrace();
        }

        for (List<Rect> group : listOfGroupsCTR) {
            Rect groupBound = boundingRectangle(group);
            drawRectangle(groupBound, new Scalar(0,150,0), false);

            if (groupBound.area() > bestRectangle.area()) {
                secondBestRectangle = bestRectangle;
                bestRectangle = groupBound;
            }
        }
        return new Rect[] {
                bestRectangle, secondBestRectangle
        };
    }

    protected Rect chooseBestRectangle(List<List<Rect>> listOfGroupsCBR) {
        Rect bestRectangle = new Rect();

        try {
            bestRectangle = boundingRectangle(listOfGroupsCBR.get(0));
        } catch (Exception exception) {
            exception.printStackTrace();
        }

        for (List<Rect> group : listOfGroupsCBR) {
            Rect groupBound = boundingRectangle(group);
            drawRectangle (groupBound, new Scalar(0,150,0), false);

            if (groupBound.area() > bestRectangle.area()) {
                bestRectangle = groupBound;
            }
        }
        return bestRectangle;
    }

    protected List<List<Rect>> groupIntoGroups(List<Rect> rects, int groupDistanceThreshold) {
        List<List<Rect>> listOfGroups = new ArrayList<>();
        List<Rect> unusedRects = new ArrayList<>(rects);

        while (!unusedRects.isEmpty()) {
            LinkedList<Rect> toProcess = new LinkedList<>();
            toProcess.add(unusedRects.remove(0));

            List<Rect> currentGroup = new ArrayList<>();

            while (!toProcess.isEmpty()) {
                Rect currentRect = toProcess.poll();
                currentGroup.add(currentRect);

                for (int i=0;i<unusedRects.size();i++) {

                    if (distance(getCenterPoint(currentRect),getCenterPoint(unusedRects.get(i)))<groupDistanceThreshold) {
                        toProcess.add(unusedRects.remove(i));
                        i--;
                    }
                }
            }
            listOfGroups.add(currentGroup);
        }
        return listOfGroups;
    }

    protected List<Rect> contoursToRects(List<MatOfPoint> contours) {
        List<Rect> rects = new ArrayList<>();
        for (MatOfPoint contour : contours) {
            rects.add(Imgproc.boundingRect(contour));
        }
        return rects;
    }

    protected List<MatOfPoint> findContours (MidnightCVColorFilter midnightCVColorFilter, Mat mat) {
        midnightCVColorFilter.process(workingMat, mat);
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(mat, contours, new Mat(),Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        return contours;
    }
    protected void cropMat(Mat input, Point t1, Point br) {
        this.tl = t1;
        this.br = br;
    }


    @Override
    public Mat processFrame(Mat input) {
        return input;
    }

    @Override
    public void onViewportTapped() {

    }
}
