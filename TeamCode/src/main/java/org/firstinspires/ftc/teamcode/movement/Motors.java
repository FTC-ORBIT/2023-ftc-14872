package org.firstinspires.ftc.teamcode.movement;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.res.HardwarePush;

public class Motors extends HardwarePush {

    static double offset = 0;

    public Motors(OpMode opMode) {
        super(opMode);
    }


    private double[] fieldCentric() {
        double x1 = opMode.gamepad1.left_stick_x;
        double y1 = -opMode.gamepad1.left_stick_y;
        double x2 = -y1 * Math.sin(-getAngle()) + x1 * Math.cos(-getAngle());
        double y2 = y1 * Math.cos(-getAngle()) + x1 * Math.sin(-getAngle());
        double rotation = opMode.gamepad1.right_trigger - opMode.gamepad1.left_trigger;
        if (opMode.gamepad1.dpad_up) gyroReset();

        return new double[]{x2, y2, rotation};
    }

    private void gyroReset() {
        offset = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }


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
            lf.setPower(lfPower / highestPower);
            rf.setPower(rfPower / highestPower);
            rb.setPower(rbPower / highestPower);
            lb.setPower(lbPower / highestPower);
        } else {
            lf.setPower(lfPower);
            rf.setPower(rfPower);
            rb.setPower(rbPower);
            lb.setPower(lbPower);
        }
    }


    public double getAngle() {
        return wrap(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle - offset);
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
