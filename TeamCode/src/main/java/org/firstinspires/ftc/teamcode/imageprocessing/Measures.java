package org.firstinspires.ftc.teamcode.imageprocessing;

import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

public class Measures {
    public static double getPixWidth(){
        //creates an object of Contours
        Contours objects = new Contours();
        //create a contourPlay MatOfPoint3f array (not list) and a boundRect array
        MatOfPoint2f[] contourPoly = new MatOfPoint2f[objects.contoursSize];
        Rect[] boundRect = new Rect[objects.contoursSize];
        //getting the largest contour corners (tl = top left & br = below right )
        Imgproc.approxPolyDP(new MatOfPoint2f(objects.contours.get(objects.maxValIdxClone).toArray()),contourPoly[objects.maxValIdxClone],3,true);
        boundRect[objects.maxValIdxClone] = Imgproc.boundingRect(new MatOfPoint(contourPoly[objects.maxValIdxClone]));
        //calculating the contour width
        double pixWidth = boundRect[objects.maxValIdxClone].br().y - boundRect[objects.maxValIdxClone].tl().y;
        return pixWidth;
    }
    //i think its 9 pixels/mm
    public static double distFromObj(double objPixelsWidth) {
        double distance = (Constants.realPoleWidth * Constants.focalLength) / objPixelsWidth;
        return distance;
    }
}
