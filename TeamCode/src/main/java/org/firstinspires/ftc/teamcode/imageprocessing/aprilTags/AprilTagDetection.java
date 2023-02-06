package org.firstinspires.ftc.teamcode.imageprocessing.aprilTags;

import org.openftc.easyopencv.OpenCvCamera;

import java.util.ArrayList;

public class AprilTagDetection {

    ATConstants atConstants = new ATConstants();
    static AprilTagDetectionPipline aprilTagDetectionPipeline;

    static org.openftc.apriltag.AprilTagDetection tagOfInterest = null;

    public static ParkingSpot findTag(OpenCvCamera camera) {

        camera.setPipeline(aprilTagDetectionPipeline);

        ArrayList<org.openftc.apriltag.AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if (currentDetections.size() != 0) {

            for (org.openftc.apriltag.AprilTagDetection tag : currentDetections) {
                if (tag.id == ATConstants.leftTagNum || tag.id == ATConstants.middleTagNum || tag.id == ATConstants.rightTagNum) {
                    tagOfInterest = tag;
                    break;
                }
            }
        }

        if (tagOfInterest.id == ATConstants.leftTagNum) {
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
