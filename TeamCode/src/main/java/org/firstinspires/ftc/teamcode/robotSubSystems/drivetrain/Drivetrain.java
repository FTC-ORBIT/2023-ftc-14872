package org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.teamcode.sensors.Gyro;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.utils.Vector;

public class Drivetrain {

    private static final DcMotor[] motors = new DcMotor[4];

    public static Vector lastVelocity = getVelocity_FieldCS();

    public static void init(HardwareMap hardwareMap) {
        //if (GlobalData.isAutonomous) drive = new SampleMecanumDrive(hardwareMap);
        motors[0] = hardwareMap.get(DcMotor.class, "lf");
        motors[1] = hardwareMap.get(DcMotor.class, "rf");
        motors[2] = hardwareMap.get(DcMotor.class, "lb");
        motors[3] = hardwareMap.get(DcMotor.class, "rb");

        for (final DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
    public static void operate(Vector velocity_W) {
        final float robotAngle = (float) Math.toRadians(Gyro.getAngle());
        Vector velocity_FieldCS_W;
        if (!GlobalData.isAutonomous){
            //field centric
            //TODO: check if you need to insert the inverse angle into rotation function
            velocity_FieldCS_W = velocity_W.rotate(-robotAngle);
        } else {
            velocity_FieldCS_W = velocity_W;
        }
        drive(velocity_FieldCS_W);
    }

    //TODO: write the velocity calculation
    public static Vector getVelocity_FieldCS() {
        return null;
    }

    public static Vector getAcceleration() {
        Vector currentVelocity = getVelocity_FieldCS();

        Vector deltaVelocity = currentVelocity.subtract(lastVelocity);
        Vector acceleration = deltaVelocity.scale(1 / GlobalData.deltaTime);

        lastVelocity = currentVelocity;
        return acceleration;
    }

    public static void stop() {
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
    }

    public static void drive(Vector drive) {
        final double lfPower = drive.y + drive.x + drive.r;
        final double rfPower = drive.y - drive.x - drive.r;
        final double lbPower = drive.y - drive.x + drive.r;
        final double rbPower = drive.y + drive.x - drive.r;
        double highestPower = 1;
        final double max = Math.max(Math.abs(lfPower),
                Math.max(Math.abs(lbPower), Math.max(Math.abs(rfPower), Math.abs(rbPower))));
        if (max > 1)
            highestPower = max;
        motors[0].setPower((lfPower / highestPower));
        motors[1].setPower((rfPower / highestPower));
        motors[2].setPower((lbPower / highestPower));
        motors[3].setPower((rbPower / highestPower));
    }
}
