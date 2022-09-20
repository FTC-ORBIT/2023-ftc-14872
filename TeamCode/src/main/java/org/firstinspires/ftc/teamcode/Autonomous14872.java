package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.imageprocessing.RodDetector;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "Autonomous14872")
public class Autonomous14872 extends LinearOpMode {
    OpenCvCamera camera;
    @Override
    public void runOpMode() throws InterruptedException {
        //initiates the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //gets the camera name from the int in the driver station
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "camera");
        //to get the webcam view
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        //creates an object to the telemetry from the file rod detector
        RodDetector detector = new RodDetector(telemetry);
        //sets the pipeline
        camera.setPipeline(detector);
        //opening the camera
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            //live streaming
            public void onOpened()
            {
                camera.startStreaming(320,240,OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        waitForStart();

        while (opModeIsActive()){
            FtcDashboard.getInstance().startCameraStream(camera,60);
        }
        switch (detector.getRodIn()) {
            case LEFT:
                //while left
            break;
            case RIGHT:
                //while right
            break;
            case MIDDLE:
                //while middle
            break;
        }
    }

}
