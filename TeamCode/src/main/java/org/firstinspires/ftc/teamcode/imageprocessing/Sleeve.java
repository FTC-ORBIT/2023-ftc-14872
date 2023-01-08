package org.firstinspires.ftc.teamcode.imageprocessing;

import org.firstinspires.ftc.teamcode.sensors.ColorSensorV3;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class Sleeve {
    ColorSensorV3 colorSensorV3 = new ColorSensorV3();
    Mat rMat = new Mat();
    Mat gmat = new Mat();
    Mat bMat = new Mat();

    public double getRDouble(Mat mat) {
        final Rect RED_ROI = new Rect(Constants.tlRoi,Constants.brRoi);
        //Imgproc.cvtColor(mat,mat,Constants.binary);
        Core.inRange(mat,Constants.lowRHSV,Constants.highRHSV,mat);
        Mat redArea = mat.submat(RED_ROI);
            Imgproc.rectangle(mat,RED_ROI,Constants.White);
        double value =  Core.sumElems(redArea).val[0] / RED_ROI.area() / 255;
        //if (value > Constants.percentage) {return 1;}else {return 0;}
        return value;
    }

    public double getGDouble(Mat mat){
        final Rect GREEN_ROI = new Rect(Constants.tlRoi,Constants.brRoi);
        //Imgproc.cvtColor(mat,mat,Constants.binary);
        Core.inRange(mat,Constants.lowGHSV,Constants.highGHSV,mat);
        Mat greenArea = mat.submat(GREEN_ROI);
            Imgproc.rectangle(mat,GREEN_ROI,Constants.White);
        double value = Core.sumElems(greenArea).val[0] / GREEN_ROI.area() / 255;
        //if (value > Constants.percentage) {return 2;}else {return 0;}
        return value;
    }

    public double getBDouble(Mat mat){

        final Rect BLUE_ROI = new Rect(Constants.tlRoi,Constants.brRoi);
        //Imgproc.cvtColor(mat,mat,Constants.binary);
        Core.inRange(mat,Constants.lowBHSV,Constants.highBHSV,mat);
        Mat blueArea = mat.submat(BLUE_ROI);
            Imgproc.rectangle(mat,BLUE_ROI,Constants.White);
        double value =  Core.sumElems(blueArea).val[0] / BLUE_ROI.area() / 255;
        //if (value > Constants.percentage) {return 3;}else{return 0;}
        return value;
    }

    public double hahahah(Mat mat) {
        int red = 0; int green = 0; int blue = 0; int colorInInt = 0;
        Imgproc.cvtColor(mat,mat,Constants.binary);
        rMat = mat; gmat = mat; bMat = mat;
        final Rect RED_ROI = new Rect(Constants.tlRoi,Constants.brRoi);
        final Rect GREEN_ROI = new Rect(Constants.tlRoi,Constants.brRoi);
        final Rect BLUE_ROI = new Rect(Constants.tlRoi,Constants.brRoi);

        Core.inRange(rMat,Constants.lowRHSV,Constants.highRHSV,rMat);
        Core.inRange(gmat,Constants.lowGHSV,Constants.highGHSV,gmat);
        Core.inRange(bMat,Constants.lowBHSV,Constants.highBHSV,bMat);

        Mat redArea = rMat.submat(RED_ROI);
        Mat greenArea = gmat.submat(GREEN_ROI);
        Mat blueArea = bMat.submat(BLUE_ROI);

        double rvalue =  Core.sumElems(redArea).val[0] / RED_ROI.area() / 255 - -0.019899999999999998;
        double gvalue = Core.sumElems(greenArea).val[0] / GREEN_ROI.area() / 255 -0.019899999999999998;
        double bvalue =  Core.sumElems(blueArea).val[0] / BLUE_ROI.area() / 255 -0.019899999999999998;

        if (rvalue > gvalue && rvalue > bvalue) { colorInInt = 1; }
        if (gvalue > rvalue && gvalue > bvalue) { colorInInt = 2; }
        if (bvalue > gvalue && bvalue > rvalue) { colorInInt = 3; }
        return rvalue;
    }


/*
    public int mostFound(Mat mat) {
        return colorSensorV3.color(
                getRDouble(mat),
                getGDouble(mat),
                getBDouble(mat)
        );
    }

 */
}
