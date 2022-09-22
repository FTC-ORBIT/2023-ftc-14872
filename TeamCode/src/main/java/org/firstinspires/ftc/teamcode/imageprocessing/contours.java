package org.firstinspires.ftc.teamcode.imageprocessing;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
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
        Scalar lowHSV = new Scalar(18 , 55 , 100);
        Scalar highHSV = new Scalar(32, 255, 255);
        Core.inRange(mat, lowHSV, highHSV, mat);
        //finding the contours
        List<MatOfPoint> contours = new ArrayList<>();
        //draw the contour
        Mat hierarchy = new Mat();
        Imgproc.findContours(mat, contours, hierarchy , Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        Imgproc.drawContours(input, contours, -1, new Scalar(255, 0, 0));

        return input;
    }
}