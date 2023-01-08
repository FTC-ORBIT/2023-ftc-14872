package org.firstinspires.ftc.teamcode.imageprocessing;



import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;

public class Constants {
    //HSV constants
    public static Scalar lowYHSV = new Scalar(18 , 100 , 100);
    public static Scalar highYHSV = new Scalar(34 , 255 , 255);
    //Scalar (color) constants
    public static Scalar Green = new Scalar(0 , 255 , 0);
    public static Scalar White = new Scalar(255 , 255 , 255);
    public static Scalar Red = new Scalar(255 , 0 , 0);
    public static Scalar Blue = new Scalar(0 , 0 , 255);
    //Turning image into binary (black & white pixels)
    public static int binary = Imgproc.COLOR_RGB2HSV;
    //Blur size radius
    public static Size BlurRadius = new Size(3,3);
    //camera
    public static OpenCvCamera camera;

    //TODO: find values
    public static Point tlRoi = new Point(0,0);
    public static Point brRoi = new Point(200,200);

    public static Scalar lowRHSV = new Scalar(0 , 125 , 125);
    public static Scalar highRHSV = new Scalar(13 , 255 , 255);
    public static Scalar lowGHSV = new Scalar(35 , 69 , 147);
    public static Scalar highGHSV = new Scalar(135 , 220 , 255);
    public static Scalar lowBHSV = new Scalar(105 , 0 , 0);
    public static Scalar highBHSV = new Scalar(180 , 255 , 175);

    public static double percentage = 0.45;

}
