package Library17822.MidnightCV.filters;

import org.opencv.core.Mat;

/**
 * Created by Victo on 1/1/2018.
 */

public abstract class MidnightCVColorFilter {
    public abstract void process(Mat input, Mat mask);
}
