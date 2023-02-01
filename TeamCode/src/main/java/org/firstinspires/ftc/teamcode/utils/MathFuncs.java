package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.teamcode.robotData.Constants;

public final class MathFuncs {

    public static double deadBand(final double val, final double xAtZero, final double xAtOne) {
        final double returnVal = (Math.abs(val) - xAtZero) / (xAtOne - xAtZero);
        return range(0, 1, returnVal) * Math.signum(val);
    }

    public static double pointAndSlopeLinear(final Vector point, final double slope, final double value) {
        return slope * (value - point.x) + point.y;
    }

    public static double twoPointsLinear(final Vector pointA, final Vector pointB, final double value) {
        return (pointA.x == pointB.x) ? Constants.INF
                : pointAndSlopeLinear(pointA, (pointA.y - pointB.y) / (pointA.x - pointB.x), value);
    }

    public static double deadBand(final Vector point, final double slope, final double minVal, final double maxVal,
            final double value) {
        return range(minVal, maxVal, pointAndSlopeLinear(point, slope, value));
    }

    public static double deadBand(final Vector a, final Vector b, final double value) {
        final double maxValue = Math.max(a.y, b.y);
        final double minValue = Math.min(a.y, b.y);
        return range(minValue, maxValue, twoPointsLinear(a, b, value));
    }

    public static double range(final double lowerBound, final double upperBound, final double value) {
        return Math.min(Math.max(lowerBound, value), upperBound);
    }

    public static double limit(final double bound, final double value) {
        return range(-Math.abs(bound), Math.abs(bound), value);
    }

    public static boolean inRange(final double value, final double lowerBound, final double upperBound) {
        return value <= upperBound && value >= lowerBound;
    }

    public static boolean inTolerance(final double value, final double wantedValue, final double tolerance) {
        return inRange(value, wantedValue - tolerance, wantedValue + tolerance);
    }

    public static double relativeDifference(final double value, final double reference) {
        return Math.abs((value - reference) / reference);
    }

    private static final int[] factorials = { 1, 1, 2, 6 };

    public static double hypotenuse(final double a, final double b) {
        return Math.sqrt(a * a + b * b);
    }

    public static Vector calcPointOnUnitCircle(double angle) {
        return new Vector(Math.cos(angle) , Math.sin(angle));
    }
    public static double calcAngleOnUnitCircle(Vector vector){
        return Math.atan2(vector.x, vector.y);
    }

    public static double smootherStep(double edge0, double edge1, double x) {
        // Scale, and clamp x to 0..1 range
        x = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
        // Evaluate polynomial
        return x * x * x * (x * (x * 6 - 15) + 10);
    }

    public static double clamp(double x, double lowerlimit, double upperlimit) {
        if (x < lowerlimit)
            x = lowerlimit;
        if (x > upperlimit)
            x = upperlimit;
        return x;
    }

}