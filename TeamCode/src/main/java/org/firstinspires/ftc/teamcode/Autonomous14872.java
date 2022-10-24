package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.imageprocessing.contours;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Autonomous14872")
public class Autonomous14872 extends LinearOpMode {
    OpenCvCamera camera;
    @Override
    public void runOpMode() throws InterruptedException {
        //initiates the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //camera name from init in the Driver Station
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "camera");
        //camera monitor view
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        //creates an object to show the camera view in the Telemetry area
        contours contours = new contours(telemetry);
        //sets the pipeline
        camera.setPipeline(contours);
        //opening the camera
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            //live streaming
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
                //it will be run if the camera could not be opened
            }
        });
        waitForStart();
        //Start streaming to FTC Dashboard
        while (opModeIsActive()){ FtcDashboard.getInstance().startCameraStream(camera,60); }
    }
}
