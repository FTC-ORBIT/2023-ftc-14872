package org.firstinspires.ftc.teamcode.imageprocessing.camera;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.imageprocessing.ImgprocConstants;
import org.firstinspires.ftc.teamcode.imageprocessing.Pipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;

public class Camera {

    private final OpenCvCamera camera;

    public Camera(HardwareMap hardwareMap) {
        //Initiates the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //Camera name from init in the Driver Station
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "camera");
        //Camera monitor view
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        //Creates an object to show the camera view in the Telemetry area
        Pipeline pipeline = new Pipeline();
        //Sets the pipeline
        camera.setPipeline(pipeline);
        //Opening the camera
        ImgprocConstants.camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            //live streaming
            public void onOpened() {
                camera.startStreaming(320, 240);
            }

            @Override
            public void onError(int errorCode) {
                //it will be run if the camera could not be opened
            }
        });
    }

    public void setPipeline(OpenCvPipeline pipeline){
        camera.setPipeline(pipeline);
    }

    public OpenCvCamera get(){
        return camera;
    }
}
