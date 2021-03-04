package Library17822.MidnightCV.scoring;

import org.opencv.core.Mat;

/**
 * Created by Victo on 9/10/2018.
 */

public abstract class MidnightCVScorer {
    public abstract double calculateScore(Mat input);
}
