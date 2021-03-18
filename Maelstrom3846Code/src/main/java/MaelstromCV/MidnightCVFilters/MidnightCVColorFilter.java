package MaelstromCV.MidnightCVFilters;

import org.opencv.core.Mat;

public abstract class MidnightCVColorFilter {
    public abstract void process(Mat input, Mat mask);
}
