package org.firstinspires.ftc.teamcode.robotSubSystems.camera;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.imageprocessing.Pipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class Camera {

    public static void init(HardwareMap hardwareMap) {
        //initiates the camera
        OpenCvCamera camera = CameraConstants.camera;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //camera name from init in the Driver Station
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "camera");
        //camera monitor view
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        //creates an object to show the camera view in the Telemetry area
        Pipeline pipeline = new Pipeline();
        //sets the pipeline
        camera.setPipeline(pipeline);
        //opening the camera
        OpenCvCamera finalCamera = camera;
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            //live streaming
            public void onOpened() {
                finalCamera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                //it will be run if the camera could not be opened
            }
        });
    }
}
