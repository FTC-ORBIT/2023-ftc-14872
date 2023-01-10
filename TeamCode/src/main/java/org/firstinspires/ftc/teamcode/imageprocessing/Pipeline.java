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
        //Assigning the value input (live vid) to the variable material
        //material = input;
        //Turn image to binary image (black & white pixels)
        //Imgproc.cvtColor(input, input, Constants.binary);
        //blur (to remove noise)
        //Imgproc.blur(material, material, Constants.BlurRadius);
        //Calling findRodCenter function from the RodFinder class
        //RodDetector.findRodCenter();
        sleeve.hello(input);
        //sleeve.hahahah(input);
        //int someonef = colorSensorV3.color(sleeve.getRDouble(input),sleeve.getGDouble(input),sleeve.getBDouble(input));
        //System.out.println(someonef);


        //System.out.println(sleeve.getBDouble(input) + " , " + sleeve.getGDouble(input) + " , " + sleeve.getBDouble(input));
        return input;
    }
    //Cloning the material (input)
    public static Mat getClonedMat() {return material.clone();}
    //Creates a public function so we can use the material (input) in every place needed
    public static Mat getMat() {return material;}
}
