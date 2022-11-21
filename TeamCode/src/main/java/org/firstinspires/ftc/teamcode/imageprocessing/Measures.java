package org.firstinspires.ftc.teamcode.imageprocessing;

import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

public class Measures {
    public static double getPixWidth(MatOfPoint contour,int maxValIdx){
        //creates an object of Contours
        Contours objects = new Contours();
        //create a contourPlay MatOfPoint3f array (not list) and a boundRect array
        MatOfPoint2f contourPoly = new MatOfPoint2f();
        Rect boundRect;
        //getting the largest contour corners (tl = top left & br = below right )
        Imgproc.approxPolyDP(new MatOfPoint2f(contour.toArray()),contourPoly,3,true);
        boundRect = Imgproc.boundingRect(new MatOfPoint(contourPoly));
        //calculating the contour width
        double pixWidth = boundRect.br().y - boundRect.tl().y;
        return pixWidth;
    }
    //i think its 9 pixels/mm
    public static double distFromObj(double objPixelsWidth) {
        double distance = (Constants.realPoleWidth * Constants.focalLength) / objPixelsWidth;
        return distance;
    }
}
