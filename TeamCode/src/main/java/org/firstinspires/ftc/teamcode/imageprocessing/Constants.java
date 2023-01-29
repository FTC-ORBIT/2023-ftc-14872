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
    public static Scalar Red = new Scalar(255 , 0 , 0);
    //Turning image into binary (black & white pixels)
    public static int binary = Imgproc.COLOR_RGB2HSV;
    //Blur size radius
    public static Size BlurRadius = new Size(3,3);
    //camera
    public static OpenCvCamera camera;

    //TODO: find values
    public static Point tlRoi = new Point(65,80);
    public static Point brRoi = new Point(170,180);
}
