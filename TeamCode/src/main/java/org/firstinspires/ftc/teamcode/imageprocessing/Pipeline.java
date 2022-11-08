package org.firstinspires.ftc.teamcode.imageprocessing;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class Pipeline extends OpenCvPipeline {

    private static Mat material;

    @Override
    public Mat processFrame(Mat input) {

        material = input;
        Imgproc.cvtColor(input, material, Constants.binary);
        //blur (to remove noise)
        Imgproc.blur(material, material, Constants.BlurRadius);
        RodFinder.findRod();

        return material;
    }

    public static Mat getClonedMat(){return material.clone();}

    public static Mat getMat(){return material;}
}
