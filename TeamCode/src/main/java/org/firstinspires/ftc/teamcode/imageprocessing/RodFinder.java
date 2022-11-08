package org.firstinspires.ftc.teamcode.imageprocessing;

public class RodFinder {
    public static void findRod(){
        Contours.getCenter(
                Contours.getBiggest(
                        Contours.getFromMat(Pipeline.getClonedMat(), Constants.lowHSV, Constants.highHSV)));
    }

}