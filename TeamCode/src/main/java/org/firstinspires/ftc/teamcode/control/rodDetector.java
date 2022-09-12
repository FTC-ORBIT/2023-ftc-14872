package org.firstinspires.ftc.teamcode.control;

import android.location.Location;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class rodDetector extends OpenCvPipeline {
    //creates a telemetry
    Telemetry telemetry;
    //creates a mat object
    Mat mat = new Mat();
    //sets the telemetry
    public rodDetector(Telemetry t) {
        telemetry = t;
    }
    final Rect LEFT_AREA = new Rect(
            //two points to the square to be in
            new Point(),
            new Point());
    final Rect RIGHT_AREA = new Rect(
            //two points to the square to be in
            new Point(),
            new Point());
    //0.4 = 40% yellow (the rod is yellow so at list 40%)
    double rodFoundIn = 0.4;
    //creates an enum class with a of the location that the rod can be in
    public enum rodIn { RIGHT, LEFT, MIDDLE, NOT_FOUND}
    private rodIn RodIn;

    @Override
    public Mat processFrame(Mat input) {
        //set the filter and filter color censoring
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(23 , 50 ,70);
        Scalar highHSV = new Scalar(32, 255, 255);
        Core.inRange(mat, lowHSV, highHSV, mat);

        //creates a right & left submat
        Mat right = mat.submat(RIGHT_AREA);
        Mat left = mat.submat(LEFT_AREA);

        //the percentage of the sides the val is 0  because there is 1 channel
        double rightPercentage = Core.sumElems(right).val[0] /RIGHT_AREA.area() / 255 ;
        double leftPercentage = Core.sumElems(right).val[0] / LEFT_AREA.area() / 255 ;

        //release the right and left mats
        right.release();
        left.release();

        //prints the data on the telemetry board
        telemetry.addData("Right percentage" ,Math.round(rightPercentage*100) + "%");
        telemetry.addData("Left percentage", Math.round(leftPercentage*100)+"%");

        //checks if rod is in the area
        boolean rodRight = rightPercentage > rodFoundIn;
        boolean rodLeft = leftPercentage > rodFoundIn;

        //checks if the rod is in the middle or at 1 of the sides

        //rod is at the middle:
        if ((rodRight && rodLeft) == true) { RodIn = rodIn.MIDDLE; telemetry.addData("rodLocation: ", "middle"); }
        //rod not found
        if (rodRight && rodLeft == false) { RodIn = rodIn.NOT_FOUND; telemetry.addData("rodLocation: ", "not found");}
        //rod is at the right side or the lift side
        if (rodRight) { RodIn = rodIn.LEFT; telemetry.addData("rodLocation: ", "left");}else{ RodIn = rodIn.RIGHT; telemetry.addData("rodLocation: ", "right");}

        //updates the telemetry stats
        telemetry.update();

        //converting box image to gray
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2BGR);

        //colors for every box when they found/not found a rod
        Scalar colorFound = new Scalar(0 ,255 ,0);
        Scalar colorNotFound = new Scalar(255,0,0);
        //setting the colors
        Imgproc.rectangle(mat, RIGHT_AREA, RodIn == rodIn.RIGHT ? colorFound:colorNotFound);
        Imgproc.rectangle(mat, LEFT_AREA, RodIn == rodIn.LEFT ? colorFound:colorNotFound);
        //returning mat
        return mat;
    }
    public rodIn getRodIn() {
        return RodIn;
    }
}
