package org.firstinspires.ftc.teamcode.imageprocessing;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.PipedWriter;

public class Pipeline extends OpenCvPipeline {
    //Creates a static Mat variable
    private static Mat material;

    @Override
    public Mat processFrame(Mat input) {
        //Assigning the value input (live vid) to the variable material
        material = input;
        //Turn image to binary image (black & white pixels)
        Imgproc.cvtColor(input, material, Constants.binary);
        //blur (to remove noise)
        Imgproc.blur(material, material, Constants.BlurRadius);
        //Calling findRodCenter function from the RodFinder class
        //RodDetector.findRodCenter();
        return material;
    }
    //Cloning the material (input)
    public static Mat getClonedMat() {return material.clone();}
    //Creates a public function so we can use the material (input) in every place needed
    public static Mat getMat() {return material;}
}
