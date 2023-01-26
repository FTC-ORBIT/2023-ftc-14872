package org.firstinspires.ftc.teamcode.imageprocessing;

import org.firstinspires.ftc.teamcode.sensors.ColorSensorV3;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class Sleeve {
    public static int mostColorInRect(Mat mat) {
        final Rect ROI = new Rect(Constants.tlRoi,Constants.brRoi);
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
                if (red > 150 && green > 150 && blue > 150) { whiteCount++; }
            }
        }
        // Find the most common color
        return mostCommonColor(redCount,greenCount,whiteCount);
    }

    public static int mostCommonColor(int redCount, int greenCount, int whiteCount) {
        int mostCommonColor = 0;
        int maxCount = Math.max(redCount, Math.max(whiteCount, greenCount));
        if (maxCount == redCount) {
            mostCommonColor = 1;
        } else if (maxCount == greenCount) {
            mostCommonColor = 2;
        }
        else if (maxCount == whiteCount) {
            mostCommonColor = 3;
        }
        return mostCommonColor;
    }


    public static Scalar pixels(Mat mat) {
        double[] pixel = mat.get(11,30);
        Scalar pixColor = new Scalar(pixel);
        final Rect ROI = new Rect(10,10,20,20);
        mat.submat(ROI).setTo(pixColor);
        return pixColor;
    }



}
