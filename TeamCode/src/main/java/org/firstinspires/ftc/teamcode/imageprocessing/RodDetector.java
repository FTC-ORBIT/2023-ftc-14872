package org.firstinspires.ftc.teamcode.imageprocessing;

import android.util.Pair;

import org.opencv.core.Point;

public class RodDetector {
    //Creating findRodCenter function
    public static void findRodCenter(){
        //Calls the getCenter function from the Contours class (to find the biggest contour's center)
        Pair<Point, Double> pair =  Contours.getCenter(
                //Calls the getBiggestContour function from the Contours class (to find the biggest contour)
                Contours.getBiggestContour(
                        //Calls the getContour function from the Contours class and assigning its values
                        Contours.getContour(Pipeline.getClonedMat(), Constants.lowHSV, Constants.highHSV)
                )
        );
    }
}