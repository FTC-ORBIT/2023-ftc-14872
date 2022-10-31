package org.firstinspires.ftc.teamcode.imageprocessing;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class Constants {
    //HSV constants
    public static Scalar lowHSV = new Scalar(18 , 100 , 100);
    public static Scalar highHSV = new Scalar(34, 255, 255);
    //Scalar (color) constants
    public static Scalar Green = new Scalar(0, 255,0);
    public static Scalar White = new Scalar(255, 255,255);
    public static Scalar Red = new Scalar(255,0,0);
    //turning image into binary (black & white pixels)
    public static int binary = Imgproc.COLOR_RGB2HSV;
    //blur size radius
    public static Size BlurRadius = new Size(3,3);
}
