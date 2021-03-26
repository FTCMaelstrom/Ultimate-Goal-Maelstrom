package MaelstromCV;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

import static MaelstromCV.CameraView.Cam.PHONE;
import static MaelstromCV.CameraView.Cam.WEBCAM;
import static org.openftc.easyopencv.OpenCvCameraRotation.UPRIGHT;

/**
 * Created by Amogh Mehta
 * Project: FtcRobotController_Ultimate-Goal_prod2
 * Last Modified: 3/17/21 3:43 PM
 * Last Updated: 3/17/21 3:45 PM
 **/
public class CameraView {
    private OpenCvCamera phoneCam;
    private OpenCvWebcam webcam;
    public CVDetector detector;
    private Cam cam;

    public enum Cam {
        PHONE, WEBCAM
    }

    public CameraView(CVDetector detector, Cam cam, HardwareMap hardwareMap) {
        this.cam = cam;
        this.detector = detector;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        if (cam.equals(PHONE)) {
            phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
            phoneCam.setPipeline(detector);
        } else if (cam.equals(WEBCAM)) {
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            webcam.setPipeline(detector);
        }
    }

    public void start(OpenCvCameraRotation rotation) {
        if (cam.equals(PHONE)) {
            phoneCam.openCameraDevice();
            phoneCam.startStreaming(320, 240, rotation);
        }
        else {
            webcam.openCameraDevice();
            webcam.startStreaming(1280, 960, rotation);
        }
    }

    public void start() {
        start(UPRIGHT);
    }

    public void stop() {
        if (cam.equals(PHONE)) {
            phoneCam.stopStreaming();
            phoneCam.closeCameraDevice();
        } else if (cam.equals(WEBCAM)) {
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }
    }

    public void pause() {
        if (cam.equals(PHONE)) phoneCam.pauseViewport();
        else if (cam.equals(WEBCAM)) webcam.pauseViewport();
    }

    public void resume() {
        if (cam.equals(PHONE)) phoneCam.resumeViewport();
        else if (cam.equals(WEBCAM)) webcam.resumeViewport();
    }

    public OpenCvCamera getCamera() {
        if (cam.equals(PHONE)) return phoneCam;
        else return webcam;
    }
}
