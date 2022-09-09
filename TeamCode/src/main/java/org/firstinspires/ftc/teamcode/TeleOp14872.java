package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.movement.Motors;
import org.firstinspires.ftc.teamcode.res.Hardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "TeleOp14872")
public class TeleOp14872 extends OpMode {
    OpenCvWebcam webcam10 = null;
    Hardware hardware = new Hardware(this);
    Motors motors = new Motors(hardware);

    @Override
    public void init() {
        hardware.init();
        WebcamName WebcamName = hardwareMap.get(WebcamName.class, "webcam");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam10 = OpenCvCameraFactory.getInstance().createWebcam(WebcamName, cameraMonitorViewId);
        //webcam10.setPipeline(new examplePipeLine());
        webcam10.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
                webcam10.startStreaming(640,360, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 * (if it didn't work)
                 */
            }
        });
    }


    @Override
    public void loop() {
        motors.drive();
    }
}