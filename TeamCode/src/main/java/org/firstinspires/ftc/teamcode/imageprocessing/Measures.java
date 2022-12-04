package org.firstinspires.ftc.teamcode.imageprocessing;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;

public class Measures {
    public static double getPixWidth(MatOfPoint2f contourPoly){
        Rect boundRect;
        //getting the largest contour corners (tl = top left & br = below right )
        boundRect = Imgproc.boundingRect(new MatOfPoint(contourPoly));
        //calculating the contour width
        double pixWidth = boundRect.br().x - boundRect.tl().x;
        return pixWidth;
    }
    public static double distFromObj(double objPixelsWidth) {
        return (Constants.realPoleWidth * Constants.focalLength) / objPixelsWidth;
    }

    public static double findDistance(Mat mat, Scalar lowHSV, Scalar highHSV) {
        return distFromObj(
                    getPixWidth(
                            Contours.contourPolyList(
                                    Contours.getBiggestContour(
                                            Contours.getContour(
                                                    mat, lowHSV, highHSV
                                            )
                                    )
                            )
                    )
        );
    }
}
