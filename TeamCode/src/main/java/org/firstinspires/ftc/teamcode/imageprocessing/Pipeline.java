package org.firstinspires.ftc.teamcode.imageprocessing;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.sensors.ColorSensorV3;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Pipeline extends OpenCvPipeline {
    Telemetry telemetry;
    //Creates a static Mat variable
    private static Mat material;
    ColorSensorV3 colorSensorV3 = new ColorSensorV3();
    Sleeve sleeve = new Sleeve();
    @Override
    public Mat processFrame(Mat input) {



        return input;
    }
    //Cloning the material (input)
    public static Mat getClonedMat() {return material.clone();}
    //Creates a public function so we can use the material (input) in every place needed
    public static Mat getMat() {return material;}
}
