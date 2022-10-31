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

public class RodFinder {
    public static void findRod(){
        Contours.getCenter(
                Contours.getBiggest(
                        Contours.getFromMat(Pipeline.getMat(), Constants.lowHSV, Constants.highHSV)));
    }

}