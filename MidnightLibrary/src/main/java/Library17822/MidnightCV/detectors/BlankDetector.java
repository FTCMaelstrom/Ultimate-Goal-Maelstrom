package Library17822.MidnightCV.detectors;

import org.opencv.core.Mat;

/**
 * Created by Victo on 12/17/2017.
 */

public class BlankDetector extends MidnightCVDetector {
    @Override
    public Mat process(Mat input) {
        // Process frame
        return input;
    }

    @Override
    public void useDefaults() {
        // Add in your scorers here.
    }
}
