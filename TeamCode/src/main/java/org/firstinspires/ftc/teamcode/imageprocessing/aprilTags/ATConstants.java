package org.firstinspires.ftc.teamcode.imageprocessing.aprilTags;

import org.opencv.core.Point;

public class ATConstants {

    public static final double FEET_PER_METER = 3.28084;

    // UNITS ARE METERS
    public static double tagsize = 0.2025;
    //tag id from the 36h11 family
    public static int leftTagNum = 9;
    public static int middleTagNum = 10;
    public static int rightTagNum = 11;


    public static Point tlRoi = new Point(130,75);
    public static Point brRoi = new Point(520,485);

}
