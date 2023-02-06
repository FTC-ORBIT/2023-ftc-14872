package org.firstinspires.ftc.teamcode.imageprocessing;



import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;

public class ImgprocConstants {
    //HSV constants
    public static Scalar lowYHSV = new Scalar(18 , 100 , 100);
    public static Scalar highYHSV = new Scalar(34 , 255 , 255);
    //Scalar (color) constants
    public static Scalar Red = new Scalar(255 , 0 , 0);
    //Turning image into binary (black & white pixels)
    public static int binary = Imgproc.COLOR_RGB2HSV;
    //Blur size radius
    public static Size BlurRadius = new Size(3,3);
    //camera
    public static OpenCvCamera camera;

    //TODO: find values
    public static Point tlRoi = new Point(155, 110);
    public static Point brRoi = new Point(240,220);

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    public static double fx = 2080.281;
    public static double fy = 2072.668;
    public static double cx = 987.693;
    public static double cy = 534.467;
}
