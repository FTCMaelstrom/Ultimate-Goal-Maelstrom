package MasqVision;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.*;

import static MidnightLibrary.MidnightResources.MidnightUtils.getHardwareMap;
import static java.util.Locale.US;
import static org.openftc.easyopencv.OpenCvCameraRotation.UPRIGHT;

/**
 * Created by Keval Kataria on 6/1/2020
 */

//OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

public class MasqCamera {
    private OpenCvCamera camera;
    public MasqCVDetector detector;
    private boolean streaming = false;

    public MasqCamera(MasqCVDetector detector) {
        this.detector = detector;
        int cameraMonitorViewId = getHardwareMap().appContext.getResources().getIdentifier("cameraMonitorViewId", "id", getHardwareMap().appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        camera.setPipeline(detector);
    }

    public void start(OpenCvCameraRotation rotation) {
        camera.openCameraDevice();
        camera.startStreaming(640, 480, rotation);
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