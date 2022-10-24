package org.firstinspires.ftc.teamcode.imageprocessing;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class contours extends OpenCvPipeline {
    //creates & imports the telemetry
    Telemetry telemetry;
    public contours(Telemetry t) {
        telemetry = t;
    }
    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, constants.binary);
        //blur (to remove noise)
        Imgproc.blur(input, input, constants.BlurRadius);
        Core.inRange(mat, constants.lowHSV, constants.highHSV, mat);

        //list of contour points
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();

        //finding contours with a function from the openCV library
        Imgproc.findContours(mat, contours, hierarchy , Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        //draw contours
        Imgproc.drawContours(input, contours, -1, constants.Red);

        //framing all the requirements for the square
        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];

        //exports all of the contour vars to create a square (from the live vid)
        Rect[] boundRect = new Rect[contours.size()];
        Point[] centers = new Point[contours.size()];
        float[][] radius = new float[contours.size()][1];


        //searching for the right radius and center vector values
        // (it turning the shape into many polygons and searching for the center and radius vectors) we will apply it later
        double maxVal = 0;
        int maxValIdx = 0;
        for (int i = 0; i < contours.size(); i++) {
            //searching for the right radius and center vector values
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
            centers[i] = new Point();
            Imgproc.minEnclosingCircle(contoursPoly[i], centers[i], radius[i]);

            //drawing the actual square (it will show in white)
            Imgproc.rectangle(input, boundRect[i].tl(), boundRect[i].br(), constants.White, 2);

            //find the largest contour prototype (by the size of the contour)
            double contourArea = Imgproc.contourArea(contours.get(i));
            if (maxVal < contourArea) {
                maxVal = contourArea;
                maxValIdx = i;
            }
        }
        Point biggestCenter = centers[maxValIdx];
        Imgproc.circle(input, biggestCenter, 5 ,constants.White , -1);
        Imgproc.drawContours(input, contours, maxValIdx, constants.Green, 5);
        return input;
    }
}