package org.firstinspires.ftc.teamcode.imageprocessing;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Pipeline extends OpenCvPipeline {
    //Creates a static Mat variable
    private static Mat material;
    public static double distanceFromRod;
    @Override
    public Mat processFrame(Mat input) {
        //Turn image to binary image (black & white pixels)
        Imgproc.cvtColor(input, input, Constants.binary);
        //blur (to remove noise)
        Imgproc.blur(input, material, Constants.BlurRadius);
        //Calling findRodCenter function from the RodFinder class
        //RodDetector.findRodCenter();
        distanceFromRod = Measures.findDistance(material ,Constants.lowHSV,Constants.highHSV);
        /*
        //Assigning the value input (live vid) to the variable material
        material = input;
         */
        return material;
    }
    //Cloning the material (input)
    public static Mat getClonedMat() {return material.clone();}
    //Creates a public function so we can use the material (input) in every place needed
    public static Mat getMat() {return material;}

    public static double distanceFromRod() {return distanceFromRod;}
}
