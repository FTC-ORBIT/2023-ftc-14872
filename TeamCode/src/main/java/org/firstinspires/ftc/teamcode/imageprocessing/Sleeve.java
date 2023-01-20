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


    public int hello(Mat mat) {
        // Define the Rectangle
        final Rect ROI = new Rect(Constants.tlRoi,Constants.brRoi);
        //Rect rect = new Rect(50, 50, 100, 100);

        // Crop the image to the rectangle
        Mat croppedImage = new Mat(mat, ROI);

        // Convert the image to HSV color space
        Mat hsvImage = new Mat();
        Imgproc.cvtColor(croppedImage, hsvImage, Imgproc.COLOR_BGR2HSV);

        // Initialize the color counters
        int redCount = 0;
        int greenCount = 0;
        int blueCount = 0;

        // Iterate through the pixels in the image
        for (int i = 0; i < hsvImage.rows(); i++) {
            for (int j = 0; j < hsvImage.cols(); j++) {
                double[] pixel = hsvImage.get(i, j);
                double hue = pixel[0];
                double saturation = pixel[1];
                double value = pixel[2];

                // Check if the pixel is red
                if (hue >= 0 && hue < 13 && saturation > 125 && saturation < 255 && value > 125) {
                    redCount++;
                }
                // Check if the pixel is green
                else if (hue >= 35 && hue < 135 && saturation > 69 && saturation < 220 && value > 147) {
                    greenCount++;
                }
                // Check if the pixel is blue
                else if (hue >= 105 && hue < 180 && saturation >= 0 && value >= 0 && value < 175) {
                    blueCount++;
                }
            }
        }

        // Determine the most common color
        int mostCommonColor = 0;
        if (redCount > greenCount && redCount > blueCount) {
            //red
            mostCommonColor = 1;
        } else if (greenCount > redCount && greenCount > blueCount) {
            //green
            mostCommonColor = 2;
        } else if (blueCount > redCount && blueCount > greenCount) {
            //blue
            mostCommonColor = 3;
        }

        // Print the result
        System.out.println("The most common color in the rectangle is: " + mostCommonColor);
        return mostCommonColor;
    }

    public int mostFound(Mat mat) {
        return colorSensorV3.color(
                getRDouble(mat),
                getGDouble(mat),
                getBDouble(mat)
        );
    }

    public static int colorMaybe(Mat mat) {
        final Rect ROI = new Rect(Constants.tlRoi,Constants.brRoi);
        Mat hsvImage = new Mat();
        Mat croppedImage = new Mat(mat, ROI);
        int redCount = 0;
        int blueCount = 0;
        int greenCount = 0;
        Imgproc.cvtColor(croppedImage,hsvImage,Constants.binary);

        // Iterate through each pixel in the image
        for (int row = 0; row < hsvImage.rows(); row++) {
            for (int col = 0; col < hsvImage.cols(); col++) {
                double[] pixel = hsvImage.get(row, col);
                double hue = pixel[0];
                double saturation = pixel[1];
                double value = pixel[2];

                // Check if the pixel is within the range of the desired color
                if (hue >= 0 && hue <= 15) {
                    redCount++;
                } else if (hue >= 110 && hue <= 130) {
                    blueCount++;
                } else if (hue >= 60 && hue <= 80) {
                    greenCount++;
                }
            }
        }

        // Find the most common color
        int mostCommonColor = 0;
        int maxCount = Math.max(redCount, Math.max(blueCount, greenCount));
        if (maxCount == redCount) {
            mostCommonColor = 1;
        } else if (maxCount == blueCount) {
            mostCommonColor = 2;
        } else if (maxCount == greenCount) {
            mostCommonColor = 3;
        }
        System.out.println("The most common color is " + mostCommonColor);
        return mostCommonColor;
    }



}
