package org.firstinspires.ftc.teamcode.imageprocessing;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.LongStream;

public class Contours {
    public static List<MatOfPoint> getFromMat(Mat mat, Scalar lowHSV, Scalar highHSV){
        Core.inRange(mat, Constants.lowHSV, Constants.highHSV, mat);

        //list of contour points
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        //finding contours with a function from the openCV library
        Imgproc.findContours(mat, contours, hierarchy , Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        return contours;
    }

    public static MatOfPoint getBiggest(List<MatOfPoint> contours){

        double maxVal = 0;
        int maxValIdx = 0;
        for (int i = 0; i < contours.size(); i++) {
            //find the largest contour prototype (by the size of the contour)
            double contourArea = Imgproc.contourArea(contours.get(i));
            if (maxVal < contourArea) {
                maxVal = contourArea;
                maxValIdx = i;
            }
        }
        return contours.get(maxValIdx);
    }

    public static Point getCenter(MatOfPoint contour){
        MatOfPoint2f contourPoly = new MatOfPoint2f();

        Imgproc.approxPolyDP(new MatOfPoint2f(contour.toArray()), contourPoly, 3, true);
        Point center = new Point();
        Imgproc.minEnclosingCircle(contourPoly, center, new float[1]);

        return center;
    }
}
