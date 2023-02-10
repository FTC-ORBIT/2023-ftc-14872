package org.firstinspires.ftc.teamcode.imageprocessing.aprilTags;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.imageprocessing.ImgprocConstants;
import org.firstinspires.ftc.teamcode.imageprocessing.camera.Camera;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.ArrayList;

public class AprilTagDetection {

    public static void init(Camera camera){

        camera.setPipeline(aprilTagDetectionPipeline);
    }

    static AprilTagDetectionPipline aprilTagDetectionPipeline = new AprilTagDetectionPipline(ATConstants.tagsize, ImgprocConstants.fx, ImgprocConstants.fy, ImgprocConstants.cx, ImgprocConstants.cy);
    static org.openftc.apriltag.AprilTagDetection tagOfInterest = null;

    public static ParkingSpot findTag(Telemetry telemetry) {

        tagOfInterest = null;

        ArrayList<org.openftc.apriltag.AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if (currentDetections.size() != 0) {
            for (org.openftc.apriltag.AprilTagDetection tag : currentDetections){
                telemetry.addData("id" + tag.id,tag.id );

                if (tag.id == ATConstants.leftTagNum || tag.id == ATConstants.middleTagNum || tag.id == ATConstants.rightTagNum) {
                    tagOfInterest = tag;
                    break;
                }
            }
            telemetry.update();
        }

        if (tagOfInterest == null){
            return ParkingSpot.NONE;
        }else if (tagOfInterest.id == ATConstants.leftTagNum) {
            return ParkingSpot.LEFT;
        } else if (tagOfInterest.id == ATConstants.middleTagNum) {
            return ParkingSpot.MIDDLE;
        } else if (tagOfInterest.id == ATConstants.rightTagNum) {
            return ParkingSpot.RIGHT;
            /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        }
        return ParkingSpot.NONE;
    }
}
