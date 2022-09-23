package org.firstinspires.ftc.teamcode.imageprocessing;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class contours extends OpenCvPipeline {
    //creates a telemetry
    Telemetry telemetry;
    //creates a material object
    Mat mat = new Mat();
    //sets the telemetry
    public contours(Telemetry t) {
        telemetry = t;
    }
    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();
        //turning image to binary image with the right settings
        //binary = black & white image (the object is white)
        //in this code we are turning the color yellow to white and else to black
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        //blur (to remove noise)
        Imgproc.blur(input, input, new Size(3, 3));
        Scalar lowHSV = new Scalar(18 , 55 , 100);
        Scalar highHSV = new Scalar(32, 255, 255);
        Core.inRange(mat, lowHSV, highHSV, mat);
        //list of contour points
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        //finding contours with a function from the openCV library
        Imgproc.findContours(mat, contours, hierarchy , Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        //draw contours
        Imgproc.drawContours(input, contours, -1, new Scalar(255, 0, 0));
        //framing all the requirements for the square
        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
        //exports all of the contour vars to create a square (from the live pic)
        Rect[] boundRect = new Rect[contours.size()];
        Point[] centers = new Point[contours.size()];
        float[][] radius = new float[contours.size()][1];
        //searching for the right radius and center vector values
        // (it turning the shape into many polygons and searching for the center and radius vectors) we will apply it later
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
            centers[i] = new Point();
            Imgproc.minEnclosingCircle(contoursPoly[i], centers[i], radius[i]);
        }
        //drawing the actual square (it will show in white)
        for (int i = 0; i < contours.size(); i++) {
            Scalar color = new Scalar(255,255,255);
            Imgproc.rectangle(input, boundRect[i].tl(), boundRect[i].br(), color, 2);
        }
        return input;
    }
}