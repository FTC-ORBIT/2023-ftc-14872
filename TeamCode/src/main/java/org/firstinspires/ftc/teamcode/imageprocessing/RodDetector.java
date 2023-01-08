package org.firstinspires.ftc.teamcode.imageprocessing;

public class RodDetector {
    //Creating findRodCenter function
    public static void findRodCenter(){
        //Calls the getCenter function from the Contours class (to find the biggest contour's center)
        Contours.getCenter(
                //Calls the getBiggestContour function from the Contours class (to find the biggest contour)
                Contours.getBiggestContour(
                        //Calls the getContour function from the Contours class and assigning its values
                        Contours.getContour(Pipeline.getClonedMat(), Constants.lowYHSV, Constants.highYHSV)
                )
        );
    }
}