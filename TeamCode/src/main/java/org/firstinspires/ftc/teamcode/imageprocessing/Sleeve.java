package org.firstinspires.ftc.teamcode.imageprocessing;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class Sleeve {

    public static int color = 0;

    public int mostColorInRect(Mat mat) {

        final Rect ROI = new Rect(ImgprocConstants.tlRoi, ImgprocConstants.brRoi);
        Imgproc.rectangle(mat, ImgprocConstants.tlRoi, ImgprocConstants.brRoi, new Scalar(255, 0 , 0));
        Mat croppedImage = new Mat(mat, ROI);
        Imgproc.blur(croppedImage, croppedImage, ImgprocConstants.BlurRadius);
        int redCount = 0; int greenCount = 0; int whiteCount = 0;

        // Iterate through each pixel in the image
        for (int row = 0; row < croppedImage.rows(); row++) {
            for (int col = 0; col < croppedImage.cols(); col++) {
                double[] pixel = croppedImage.get(row, col);
                Scalar pixScalar = new Scalar(pixel);

                double red = pixScalar.val[0];
                double green = pixScalar.val[1];
                double blue = pixScalar.val[2];


                if (red > 23 && red < 98 && green > 73 && green < 210 && blue > 39 && blue < 163) { greenCount++;
                }
                else if (red > 138 && red < 255 && green > 0 && green < 150 && blue > 67 && blue < 146) { redCount++;}
                else if (red > 133 && red < 255 && green > 158 && green < 255 && blue > 165 && blue < 255) { whiteCount++;
                }
            }
        }
        /*
        Telemetry telemetry = Autonomous14872.telemetry1;
           telemetry.addData("redCount", redCount - 200);
           telemetry.addData("greenCount", greenCount);
           telemetry.addData("whiteCount", whiteCount - 2600);
           telemetry.addData("color", color);
           telemetry.update();

         */

        // Find the most common color
        return mostCommonColor(redCount - 200 , greenCount, whiteCount - 2600 );
    }

    public int mostCommonColor(int redCount, int greenCount, int whiteCount) {
        int maxCount = Math.max(redCount, Math.max(whiteCount, greenCount));
        if(maxCount == redCount) return 1;
        if(maxCount == greenCount) return 2;
        return 3;
    }

}
