package org.firstinspires.ftc.teamcode.imageprocessing;

import androidx.annotation.ColorLong;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class RodDetector extends OpenCvPipeline {
    //creates a telemetry
    Telemetry telemetry;
    //creates a material object
    Mat mat = new Mat();
    //sets the telemetry
    public RodDetector(Telemetry t) {
        telemetry = t;
    }
    final Rect LEFT_AREA = new Rect(
            //two points for the square to be in 60,35 //// 120,75
            new Point(0,0),
            new Point(100,240));
    final Rect RIGHT_AREA = new Rect(
            //two points for the square to be in
            new Point(230,0),
            new Point(320,240));
    final Rect MIDDLE_AREA = new Rect(
            new Point(120,0),
            new Point(220,240));
    //0.4 = 40% yellow (the rod is at least 405 yellow)
    double rodFoundIn = 0.4;
    //creates an enum class with a of the location that the rod can be in
    public enum rodIn { RIGHT, LEFT, MIDDLE, NOT_FOUND}
    private rodIn RodIn;

    @Override
    public Mat processFrame(Mat input) {
        //set the filter and filter color censoring
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        //low color setting
        Scalar lowHSV = new Scalar(18 , 55 , 100);
        //high color setting
        Scalar highHSV = new Scalar(32, 255, 255);
        Core.inRange(mat, lowHSV, highHSV, mat);

        //creates a right & left submat
        Mat right = mat.submat(RIGHT_AREA);
        Mat left = mat.submat(LEFT_AREA);
        Mat middle = mat.submat(MIDDLE_AREA);

        //the percentage of the sides the val is 0  because there is 1 channel
        double rightPercentage = Core.sumElems(right).val[0] / RIGHT_AREA.area() / 255 ;
        double leftPercentage = Core.sumElems(left).val[0] / LEFT_AREA.area() / 255 ;
        double middlePercentage = Core.sumElems(middle).val[0] / MIDDLE_AREA.area() / 255;

        //release the right and left and middle materials
        right.release();
        left.release();
        middle.release();

        //prints the data on the telemetry board
        telemetry.addData("Right percentage" ,Math.round(rightPercentage*100) + "%");
        telemetry.addData("Left percentage", Math.round(leftPercentage*100)+"%");
        telemetry.addData("middle Percentage", Math.round(middlePercentage*100)+"%");

        //checks if rod is in the area
        boolean rodRight = rightPercentage > rodFoundIn;
        boolean rodLeft = leftPercentage > rodFoundIn;
        boolean rodMiddle = middlePercentage > rodFoundIn;
        //checks if the rod is in the middle or 1 of the sides

        //rod is at the middle:
        if ((rodMiddle)) { RodIn = rodIn.MIDDLE; telemetry.addData("rodLocation: ", "middle"); }
        //rod not found
        if (!rodRight && !rodLeft) { RodIn = rodIn.NOT_FOUND; telemetry.addData("rodLocation: ", "not found");}
        //rod is at the right side or the left side
        if (rodRight) { RodIn = rodIn.LEFT; telemetry.addData("rodLocation: ", "left");}else{ RodIn = rodIn.RIGHT; telemetry.addData("rodLocation: ", "right");}

        //updates the telemetry stats
        telemetry.update();

        //converting box image to gray
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2BGR);

        //colors for every box when they found/not found a rod
        Scalar colorFound = new Scalar(0 ,255 ,0);
        Scalar colorNotFound = new Scalar(255,0,0);
        //setting the colors
        Imgproc.rectangle(mat, RIGHT_AREA, RodIn == rodIn.RIGHT ? colorNotFound:colorFound);
        Imgproc.rectangle(mat, LEFT_AREA, RodIn == rodIn.LEFT ? colorNotFound:colorFound);
        //Imgproc.rectangle(mat, MIDDLE_AREA,RodIn == rodIn.MIDDLE ? colorNotFound:colorFound);
        //returning mat
        return mat;
    }
    public rodIn getRodIn() {
        return RodIn;
    }
}
/*
class contours extends OpenCvPipeline {
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
        Mat hierarchey = new Mat();
        Imgproc.findContours(input, contours, hierarchey , Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE );
        Scalar color = new Scalar(0, 0, 255);
        Imgproc.drawContours(input, contours, -1, color) ;

        return input;
    }
}
*/