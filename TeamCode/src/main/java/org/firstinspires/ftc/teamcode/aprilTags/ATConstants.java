package org.firstinspires.ftc.teamcode.aprilTags;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.opencv.core.Point;

public class ATConstants {

    public static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    public static double fx = 2080.281;
    public static double fy = 2072.668;
    public static double cx = 987.693;
    public static double cy = 534.467;

    // UNITS ARE METERS
    public static double tagsize = 0.166;
    //tag id from the 36h11 family
    public static int leftTagNum = 9;
    public static int middleTagNum = 10;
    public static int rightTagNum = 11;


    public static Point tlRoi = new Point(130,75);
    public static Point brRoi = new Point(520,485);

}
