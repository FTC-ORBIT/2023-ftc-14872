package org.firstinspires.ftc.teamcode.imageprocessing;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import java.util.*;

public class Contours {
    public static List<MatOfPoint> getContour(Mat mat, Scalar lowHSV, Scalar highHSV){
        Core.inRange(mat, lowHSV, highHSV, mat);

        //List of contour points
        List<MatOfPoint> contours = new ArrayList<>();
        //Creates a hierarchy Mat object
        Mat hierarchy = new Mat();

        //finding contours with a function from the openCV library
        Imgproc.findContours(mat, contours, hierarchy , Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        //Drawing contours with a function from the openCV library
        Imgproc.drawContours(mat, contours, -1, Constants.Red, 5);
        return contours;
    }

    public static MatOfPoint getBiggestContour(List<MatOfPoint> contours){
        if (contours.isEmpty()){return new MatOfPoint();} //checks if the contour list is empty or not

        double maxVal = 0;
        int maxValIdx = 0;
        //Looping through all of the contours that are on the list "contours"
        for (int i = 0; i < contours.size(); i++) {
            //Find the largest contour prototype (by the size of the contour)
            double contourArea = Imgproc.contourArea(contours.get(i));
            if (maxVal < contourArea) {
                maxVal = contourArea;
                maxValIdx = i;
            }
        }
        return contours.get(maxValIdx);
    }

    public static Point getCenter(MatOfPoint contour){
        if (contour.empty()){return null;} //Checks if the contour list is empty or not
        MatOfPoint2f contourPoly = new MatOfPoint2f();
        //Gets all of the polygonal curves on the contour and puts them on to a list of Mat Points
        Imgproc.approxPolyDP(new MatOfPoint2f(contour.toArray()), contourPoly, 3, true);
        Point center = new Point();
        Rect boundRect;
        boundRect = Imgproc.boundingRect(Pipeline.getMat());
        //boundRect.br().x - boundRect.tl().x
        double pixWidth = 11;
        //Finds a circle of the minimum area enclosing a 2D point set (the minimum enclosing circle of a contour)
        Imgproc.minEnclosingCircle(contourPoly, center, new float[1]);
        //Drawing a circle on the center of the contour
        Imgproc.circle(Pipeline.getMat(), center, 10, Constants.Red);
        return new Point(pixWidth,0);
    }
    public static double distance() {
        double ppppp = Objects.requireNonNull(Contours.getCenter(
                //Calls the getBiggestContour function from the Contours class (to find the biggest contour)
                Contours.getBiggestContour(
                        //Calls the getContour function from the Contours class and assigning its values
                        Contours.getContour(Pipeline.getMat(), Constants.lowHSV, Constants.highHSV)
                )
        )).x;
        double pixWidth = ppppp;
        System.out.println(pixWidth);
        double constants = 26.6 * 2080;
        //double distance = constants / pixWidth;
        return constants;
    }

}
