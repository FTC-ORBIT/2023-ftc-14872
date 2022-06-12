package org.firstinspires.ftc.teamcode.recourses14872;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class HardwarePush {

    private DcMotor lf;
    private DcMotor rf;
    private DcMotor lb;
    private DcMotor rb;
    private final OpMode opMode;
    public BNO055IMU imu;
     double offset = 0;

    public HardwarePush(final OpMode opMode) {
        this.opMode = opMode;
    }

    public void init() {
        lf = opMode.hardwareMap.get(DcMotor.class, "lf");
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rf = opMode.hardwareMap.get(DcMotor.class, "rf");
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lb = opMode.hardwareMap.get(DcMotor.class, "lb");
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rb = opMode.hardwareMap.get(DcMotor.class, "rb");
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }

    public void mecanum() { //we can do extends OpMode for doing it simple
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


    public void drive(double y, double x, double r) {
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

    public double wrap(double angle) {
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