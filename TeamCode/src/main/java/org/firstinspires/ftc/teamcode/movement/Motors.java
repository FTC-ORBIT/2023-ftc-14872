package org.firstinspires.ftc.teamcode.movement;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.res.Hardware;

public class Motors {

    static double offset = 0;
    private final Hardware hardware;
    private final OpMode opMode;

    public Motors(Hardware hardware) {
        this.hardware = hardware;
        opMode = hardware.opMode;
    }


    /**
     * does all of the field centric calculations
     * @return the corrected Joystick values
     */
    private double[] fieldCentric() {
        double x1 = opMode.gamepad1.left_stick_x;
        double y1 = -opMode.gamepad1.left_stick_y;
        double x2 = -y1 * Math.sin(-getAngle()) + x1 * Math.cos(-getAngle());
        double y2 = y1 * Math.cos(-getAngle()) + x1 * Math.sin(-getAngle());
        double rotation = opMode.gamepad1.right_trigger - opMode.gamepad1.left_trigger;
        if (opMode.gamepad1.dpad_up) gyroReset();

        return new double[]{x2, y2, rotation};
    }

    /**
     * resets the gyro
     */
    private void gyroReset() {
        offset = hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }


    /**
     * enables joystick drive control on the robot
     */
    public void drive() {
        double[] values = fieldCentric();

        double x = values[0], y = values[1], r = values[2];

        final double lfPower = y + x + r;
        final double rfPower = y - x - r;
        final double lbPower = y - x + r;
        final double rbPower = y + x - r;
        final double highestPower =
                Math.max(
                        Math.abs(lfPower), Math.max(
                                Math.abs(rfPower), Math.max(
                                        Math.abs(lbPower), Math.max(
                                                Math.abs(rbPower), 1)
                                )
                        )
                );

        if (highestPower > 1) {
            hardware.lf.setPower(lfPower / highestPower);
            hardware.rf.setPower(rfPower / highestPower);
            hardware.rb.setPower(rbPower / highestPower);
            hardware.lb.setPower(lbPower / highestPower);
        } else {
            hardware.lf.setPower(lfPower);
            hardware.rf.setPower(rfPower);
            hardware.rb.setPower(rbPower);
            hardware.lb.setPower(lbPower);
        }
    }

    /**
     * getter for the gyro angle in radians.
     * from -Pi - Pi
     * @return gyro angle
     */
    public double getAngle() {
        return wrap(hardware.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle - offset);
    }

    private double wrap(double angle) {
        final double wrapped = angle % (2 * Math.PI);
        if (wrapped > Math.PI) {
            return wrapped - (2 * Math.PI);
        } else if (wrapped < -Math.PI) {
            return wrapped + (Math.PI * 2);
        } else {
            return wrapped;
        }
    }


}
