package org.firstinspires.ftc.teamcode.imageprocessing;

import org.firstinspires.ftc.teamcode.sensors.ColorSensorV3;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

public class Sleeve {
    ColorSensorV3 colorSensorV3 = new ColorSensorV3();
    final Rect RED_ROI = new Rect(Constants.tlRoi,Constants.brRoi);
    final Rect GREEN_ROI = new Rect(Constants.tlRoi,Constants.brRoi);
    final Rect BLUE_ROI = new Rect(Constants.tlRoi,Constants.brRoi);
    Mat rMat = new Mat();
    Mat gMat = new Mat();
    Mat bMat = new Mat();

    public double getRDouble(Mat mat) {
        Imgproc.cvtColor(mat,rMat,Constants.binary);
        //TODO: Find hsv values and maybe not hsv bc we are using rgb
        Core.inRange(rMat,Constants.Red,Constants.Red,rMat);
        Mat redArea = mat.submat(RED_ROI);
        return Core.sumElems(redArea).val[0] / RED_ROI.area() / 255;
    }
    public double getGDouble(Mat mat){
        Imgproc.cvtColor(mat,gMat,Constants.binary);
        //TODO: Find hsv values and maybe not hsv bc we are using rgb
        Core.inRange(gMat,Constants.Green,Constants.Green,gMat);
        Mat greenArea = mat.submat(GREEN_ROI);
        return Core.sumElems(greenArea).val[0] / GREEN_ROI.area() / 255;
    }
    public double getBDouble(Mat mat){
        Imgproc.cvtColor(mat,bMat,Constants.binary);
        //TODO: Find hsv values and maybe not hsv bc we are using rgb
        Core.inRange(bMat,Constants.Blue,Constants.Blue,bMat);
        Mat blueArea = mat.submat(BLUE_ROI);
        return Core.sumElems(blueArea).val[0] / BLUE_ROI.area() / 255;
    }
    public int mostFound(Mat mat) {
        return colorSensorV3.color(
                (int)getRDouble(mat),
                (int)getGDouble(mat),
                (int)getBDouble(mat)
        );
    }
}
