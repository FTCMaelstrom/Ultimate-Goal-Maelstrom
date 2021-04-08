package MasqVision;

import androidx.annotation.NonNull;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import static MidnightLibrary.MidnightResources.MidnightUtils.getHardwareMap;
import static java.util.Locale.US;
import static org.openftc.easyopencv.OpenCvCameraRotation.UPRIGHT;

/**
 * Created by Keval Kataria on 6/1/2020
 */

public class MasqCamera {
    public MasqCVDetector detector;
    private final OpenCvCamera camera;
    private boolean streaming = false;

    public MasqCamera(MasqCVDetector detector) {
        this.detector = detector;
        int cameraMonitorViewId = getHardwareMap().appContext.getResources().getIdentifier("cameraMonitorViewId", "id",
                getHardwareMap().appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);
        camera.setPipeline(detector);
    }

    public void start(OpenCvCameraRotation rotation) {
        camera.openCameraDevice();
        camera.startStreaming(176, 144, rotation);
        streaming = true;
    }

    public void start() {
        start(UPRIGHT);
    }

    public void stop() {
        camera.stopStreaming();
        camera.closeCameraDevice();
        streaming = false;
    }

    @NonNull
    @Override
    public String toString() {
        return String.format(US, "Webcam:\nStreaming: %s", streaming ? "Yes" : "No");
    }
}