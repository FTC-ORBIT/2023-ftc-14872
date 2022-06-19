package org.firstinspires.ftc.teamcode.movement;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.rescourses.HardwarePush;

public class Motors extends HardwarePush {

    static double offset = 0;

    public Motors(OpMode opMode) {
        super(opMode);
    }


    public void fieldCentric() {
        double x1 = opMode.gamepad1.left_stick_x;
        double y1 = -opMode.gamepad1.left_stick_y;
        double x2 = -y1 * Math.sin(-getAngle()) + x1 * Math.cos(-getAngle());
        double y2 = y1 * Math.cos(-getAngle()) + x1 * Math.sin(-getAngle());
        double rotation = opMode.gamepad1.right_trigger - opMode.gamepad1.left_trigger;
        drive(y2, x2, rotation);
        if (opMode.gamepad1.dpad_up) gyroReset();
    }

    private void gyroReset() {
        offset = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }


    private void drive(double y, double x, double r) {
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
