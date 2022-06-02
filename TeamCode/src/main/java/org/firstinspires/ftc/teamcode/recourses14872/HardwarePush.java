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

    public  HardwarePush(final OpMode opMode){
        this.opMode = opMode;
    }

    public void init(){
        lf = opMode.hardwareMap.get(DcMotor.class, "lf");
        rf = opMode.hardwareMap.get(DcMotor.class, "rf");
        lb = opMode.hardwareMap.get(DcMotor.class, "lb");
        rb = opMode.hardwareMap.get(DcMotor.class, "rb");
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }

    public void mecanum() {

        final Vector joystickVector = new Vector(gamepad1.left_stick_x, gamepad1.left_stick_y);
        final float robotAngle = (float) Math.toRadians(-getAngle());
        final Vector rotated = joystickVector.rotate(robotAngle);
        drive(rotated.y, rotated.x, gamepad1.right_trigger - gamepad1.left_trigger);
    }

    public void drive(float y, float x, float r) {
        final float lfPower = y + x + r;
        final float rfPower = y - x + r;
        final float lbPower = y - x - r;
        final float rbPower = y + x - r;

        lf.setPower(lfPower);
        rf.setPower(rfPower);
        rb.setPower(rbPower);
        lb.setPower(lbPower);
    }


    public float getAngle(){
        return wrap(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
    }

    public float wrap(float angle){
        final float wrapped = angle % 360;
        if (wrapped > 180) {
            return wrapped - 360;
        } else if (wrapped < -180) {
            return wrapped + 360;
        } else {
            return wrapped;
        }
    }
}
