package MaelstromCV.UltimateGoalSpecific;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import MaelstromCV.CVDetector;
import MaelstromCV.MidnightCVFilters.MidnightCVColorFilter;
import MaelstromCV.MidnightCVFilters.YCbCrLuminanceFilter;

import static org.opencv.core.Core.extractChannel;
import static org.opencv.imgproc.Imgproc.cvtColor;
/**
 * Created by Amogh Mehta
 * Project: FtcRobotController_Ultimate-Goal_prod2
 * Last Modified: 3/17/21 4:00 PM
 * Last Updated: 3/17/21 4:04 PM
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

    private MidnightCVColorFilter midnightCVColorFilter = new YCbCrLuminanceFilter(100);

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

                Rect topRectangle = new Rect(tl, new Point(br.x,tl.y+(br.y-tl.y)*3.0/4.0));
                Rect bottomRectangle = new Rect(new Point(tl.x, topRectangle.br().y), br);
                Rect controlRectangle = new Rect(new Point(tl.x, br.y), new Point(br.x, br.y + topRectangle.height + bottomRectangle.height));
            }
        }
        return displayMat;
    }

}
