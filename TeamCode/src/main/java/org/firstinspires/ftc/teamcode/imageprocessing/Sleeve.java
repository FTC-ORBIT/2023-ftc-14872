package org.firstinspires.ftc.teamcode.imageprocessing;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autonomous14872;
import org.firstinspires.ftc.teamcode.sensors.ColorSensorV3;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class Sleeve {

    public static int color = 0;

    public int mostColorInRect(Mat mat) {
        final Rect ROI = new Rect(Constants.tlRoi,Constants.brRoi);
        Imgproc.rectangle(mat, Constants.tlRoi, Constants.brRoi, new Scalar(255, 0 , 0));
        Mat croppedImage = new Mat(mat, ROI);
        int redCount = 0; int greenCount = 0; int whiteCount = 0;

        // Iterate through each pixel in the image
        for (int row = 0; row < croppedImage.rows(); row++) {
            for (int col = 0; col < croppedImage.cols(); col++) {
                double[] pixel = croppedImage.get(row, col);
                Scalar pixScalar = new Scalar(pixel);

                double red = pixScalar.val[0];
                double green = pixScalar.val[1];
                double blue = pixScalar.val[2];

                if (red > green && red > blue) { redCount++; }
                if (green > red && green > blue) { greenCount++; }
                if (red > 170 && green > 170 && blue > 170) { whiteCount++; }
            }
        }
        Telemetry telemetry = Autonomous14872.telemetry1;
        //telemetry.addData("redCount", redCount);
        //telemetry.addData("greenCount", greenCount - 3000);
        //telemetry.addData("whiteCount", whiteCount -2500 );
        //telemetry.addData("color", color);
        //telemetry.update();


        // Find the most common color
        return mostCommonColor(redCount + 1500,greenCount- 3000,whiteCount - 2500);
    }

    public int mostCommonColor(int redCount, int greenCount, int whiteCount) {
        int maxCount = Math.max(redCount, Math.max(whiteCount, greenCount));
        if(maxCount == redCount) return 1;
        if(maxCount == greenCount) return 2;
        return 3;
    }


    public Scalar pixels(Mat mat) {
        double[] pixel = mat.get(11,30);
        Scalar pixColor = new Scalar(pixel);
        final Rect ROI = new Rect(10,10,20,20);
        mat.submat(ROI).setTo(pixColor);
        return pixColor;
    }



}
