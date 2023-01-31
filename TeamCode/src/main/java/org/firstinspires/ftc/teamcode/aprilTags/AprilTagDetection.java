package org.firstinspires.ftc.teamcode.aprilTags;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


public class AprilTagDetection {

    private static ParkingSpot parkingSpot = ParkingSpot.LEFT;

    static OpenCvCamera camera;
    static AprilTagDetectionPipline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    static double fx = 2080.281;
    static double fy = 2072.668;
    static double cx = 987.693;
    static double cy = 534.467;

    // UNITS ARE METERS
    static double tagsize = 0.166;

    static int leftTagNum = 0;
    static int middleTagNum = 1;
    static int rightTagNum = 2;

    static org.openftc.apriltag.AprilTagDetection tagOfInterest = null;

    public static void runAprilTagDetection(LinearOpMode opMode) {
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(opMode.hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        opMode.telemetry.setMsTransmissionInterval(50);

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
                    if(tag.id == leftTagNum || tag.id == middleTagNum || tag.id == rightTagNum)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    opMode.telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest, opMode);
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

        if(tagOfInterest.id == leftTagNum) {
            parkingSpot = ParkingSpot.LEFT;
        }else if (tagOfInterest.id == middleTagNum){
            parkingSpot = ParkingSpot.MIDDLE;
        }else if (tagOfInterest.id == rightTagNum){
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
//        opMode.telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
//        opMode.telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
//        opMode.telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
//        opMode.telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//        opMode.telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//        opMode.telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

}
