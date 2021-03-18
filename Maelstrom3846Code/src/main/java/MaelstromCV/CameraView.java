package MaelstromCV;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;
/**
 * Created by Amogh Mehta
 * Project: FtcRobotController_Ultimate-Goal_prod2
 * Last Modified: 3/17/21 3:43 PM
 * Last Updated: 3/17/21 3:45 PM
 **/
public class CameraView {
    private OpenCvCamera phoneCamera;
    private OpenCvWebcam openCvWebcam;
    public CVDetector cvDetector;
    private OpenCvCameraRotation rotation;

    public enum CamSelect {
        PHONECAMBACK, PHONECAMFRONT, WEBCAM
    }

    private CamSelect cam;

    public CameraView (CVDetector detector, CamSelect camera, HardwareMap hardwareMap) {
        this.cam = camera;
        this.cvDetector = detector;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        if (cam.equals(CamSelect.PHONECAMBACK)) {
            phoneCamera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
            phoneCamera.setPipeline(detector);
        } else if (cam.equals(CamSelect.PHONECAMFRONT)) {
            phoneCamera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);
            phoneCamera.setPipeline(detector);
        } else if (cam.equals(CamSelect.WEBCAM)) {
            openCvWebcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam1"), cameraMonitorViewId);
            openCvWebcam.setPipeline(detector);
        }
    }

    public void start (OpenCvCameraRotation cameraRotation) {
        this.rotation = cameraRotation;
        if (cam.equals(CamSelect.PHONECAMBACK) || cam.equals(CamSelect.PHONECAMFRONT)) {
            phoneCamera.openCameraDevice();
            phoneCamera.startStreaming(320,240,cameraRotation);
        } else if (cam.equals(CamSelect.WEBCAM)) {
            openCvWebcam.openCameraDevice();
            openCvWebcam.startStreaming(1280, 920, cameraRotation);
        }
    }

    public void start() {
        start(OpenCvCameraRotation.UPRIGHT);
        rotation = OpenCvCameraRotation.UPRIGHT;
    }

    public void pause() {
        if (cam.equals(CamSelect.PHONECAMFRONT) || cam.equals(CamSelect.PHONECAMBACK)) {
            phoneCamera.pauseViewport();
        } else if (cam.equals(CamSelect.WEBCAM)) {
            openCvWebcam.pauseViewport();
        }
    }

    public void resume() {
        if (cam.equals(CamSelect.PHONECAMFRONT) || cam.equals(CamSelect.PHONECAMBACK)) {
            phoneCamera.resumeViewport();
        } else if (cam.equals(CamSelect.WEBCAM)) {
            openCvWebcam.resumeViewport();
        }
    }

    public void stop() {
        if (cam.equals(CamSelect.PHONECAMFRONT) || cam.equals(CamSelect.PHONECAMBACK)) {
            phoneCamera.stopStreaming();
            phoneCamera.closeCameraDevice();
        } else if (cam.equals(CamSelect.WEBCAM)) {
            openCvWebcam.stopStreaming();
            openCvWebcam.closeCameraDevice();
        }
    }

    public void pipelineSwitcher(CVDetector cameraViewDetector) {
        if (cam == CamSelect.PHONECAMFRONT || cam == CamSelect.PHONECAMBACK) {
            phoneCamera.stopStreaming();
            phoneCamera.setPipeline(cameraViewDetector);
            phoneCamera.startStreaming(320,240,rotation);
        } else if (cam == CamSelect.WEBCAM) {
            openCvWebcam.stopStreaming();
            openCvWebcam.setPipeline(cameraViewDetector);
            openCvWebcam.startStreaming(1280,920,rotation);
        }
    }

    public OpenCvCamera getCamera() {
        if (cam.equals(CamSelect.PHONECAMFRONT) || cam.equals(CamSelect.PHONECAMBACK)) {
            return phoneCamera;
        } else return openCvWebcam;
    }

}
