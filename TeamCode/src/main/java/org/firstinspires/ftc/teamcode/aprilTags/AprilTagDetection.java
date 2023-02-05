package org.firstinspires.ftc.teamcode.aprilTags;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class AprilTagDetection {
    ATConstants atConstants = new ATConstants();
    private static ParkingSpot parkingSpot = ParkingSpot.LEFT;

    static OpenCvCamera camera;
    static AprilTagDetectionPipline aprilTagDetectionPipeline;

    static org.openftc.apriltag.AprilTagDetection tagOfInterest = null;

    public static void runAprilTagDetection(LinearOpMode opMode) {
        cameraInit(opMode);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!opMode.isStarted() && !opMode.isStopRequested())
        {
            ArrayList<org.openftc.apriltag.AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(org.openftc.apriltag.AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == ATConstants.leftTagNum || tag.id == ATConstants.middleTagNum || tag.id == ATConstants.rightTagNum)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
            }
            else
            {
                opMode.telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    opMode.telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    opMode.telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest, opMode);
                }

            }

            opMode.telemetry.update();
            opMode.sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            opMode.telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest, opMode);
            opMode.telemetry.update();
        }
        else
        {
            opMode.telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            opMode.telemetry.update();
        }

        if(tagOfInterest.id == ATConstants.leftTagNum) {
            parkingSpot = ParkingSpot.LEFT;
        }else if (tagOfInterest.id == ATConstants.middleTagNum){
            parkingSpot = ParkingSpot.MIDDLE;
        }else if (tagOfInterest.id == ATConstants.rightTagNum){
            parkingSpot = ParkingSpot.RIGHT;
        }

        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
    }

    public static ParkingSpot wantedParkingSpot(){
        return parkingSpot;
    }


    static void tagToTelemetry(org.openftc.apriltag.AprilTagDetection detection , LinearOpMode opMode)
    {
        opMode.telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
    }



    public static void cameraInit(LinearOpMode opMode) {
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipline(ATConstants.tagsize, ATConstants.fx, ATConstants.fy, ATConstants.cx, ATConstants.cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) { /*on error*/ }
        });
    }



}
