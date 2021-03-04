package org.firstinspires.ftc.teamcode.Mako.Vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

import Library17822.MidnightCV.detectors.MidnightCVDetector;

/**
 * Created by Amogh Mehta on 03/03/2021 at 10:09 PM
 * Project: Ultimate-Goal_test
 */

public class CameraHandler {
    private OpenCvCamera phoneCamera;
    private OpenCvWebcam webCamera;

    public MidnightCVDetector midnightCVDetector;

    private CamSet camera;

    public enum CamSet {
        SETWEBCAM, PHONECAM
    }

    public CameraHandler(MidnightCVDetector cvDetector, CamSet camSet, HardwareMap hardwareMap) {
        this.camera = camSet;
        this.midnightCVDetector = cvDetector;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        if (camSet.equals(CamSet.PHONECAM)) {
            phoneCamera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
            phoneCamera.setPipeline(cvDetector);
        } else if (camSet.equals(CamSet.SETWEBCAM)) {
            webCamera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam #1"), cameraMonitorViewId);
            webCamera.setPipeline(cvDetector);
        }
    }

    public void start(OpenCvCameraRotation cameraRotation) {
        if (camera.equals(CamSet.PHONECAM)) {
            phoneCamera.openCameraDevice();
            phoneCamera.startStreaming(320, 240, cameraRotation);
        } else if (camera.equals(CamSet.SETWEBCAM)) {
            phoneCamera.openCameraDeviceAsync(() -> webCamera.startStreaming(1280,960,cameraRotation));
        }
    }

    public void start() {
        start(OpenCvCameraRotation.UPRIGHT);
    }

    public void stop() {
        if(camera.equals(CamSet.PHONECAM)) {
            phoneCamera.stopStreaming();
            phoneCamera.closeCameraDevice();
        } else if (camera.equals(CamSet.SETWEBCAM)) {
            webCamera.stopStreaming();
            webCamera.closeCameraDevice();
        }
    }

    public void pause() {
        if (camera.equals(CamSet.PHONECAM)) {
            phoneCamera.pauseViewport();
        } else if (camera.equals(CamSet.SETWEBCAM)) {
            webCamera.pauseViewport();
        }
    }

    public void resume() {
        if (camera.equals(CamSet.PHONECAM)) {
            phoneCamera.resumeViewport();
        } else if (camera.equals(CamSet.SETWEBCAM)) {
            webCamera.pauseViewport();
        }
    }
}
