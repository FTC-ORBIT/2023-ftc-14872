package org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.teamcode.control.PID;
import org.firstinspires.ftc.teamcode.res.Gyro;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.utils.Angle;
import org.firstinspires.ftc.teamcode.utils.MathFuncs;
import org.firstinspires.ftc.teamcode.utils.Vector;

public class Drivetrain {

    public final DcMotor[] motors = new DcMotor[4];
    //public Vector lastVelocity = getVelocity_FieldCS();

    public void init(HardwareMap hardwareMap) {
        //if (GlobalData.isAutonomous) drive = new SampleMecanumDrive(hardwareMap);
        motors[0] = hardwareMap.get(DcMotor.class, "lf");
        motors[1] = hardwareMap.get(DcMotor.class, "rf");
        motors[2] = hardwareMap.get(DcMotor.class, "lb");
        motors[3] = hardwareMap.get(DcMotor.class, "rb");

        for (final DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
    public void operate(Vector velocity_W, double rotation) {
        final double robotAngle = Math.toRadians(Gyro.getAngle());
        //TODO: check if you need to insert the inverse angle into rotation function
        drive(velocity_W.rotate(robotAngle), rotation);
    }

    //TODO: make usable
    /*
    private double[] wheelsPrevPosCm = new double[4];
    private double[] wheelsCurrentSpeedCm = new double[4];
    public Vector getVelocity_FieldCS() {
        for (int i = 0; i < 4; i++){
            wheelsCurrentSpeedCm[i] = ((motors[i].getCurrentPosition() / DrivetrainConstants.ticksPerRev) * DrivetrainConstants.cmPerWheelRev - wheelsPrevPosCm[i]) / GlobalData.deltaTime;
            wheelsPrevPosCm[i] = motors[i].getCurrentPosition();
        }
        Vector velocityField = new Vector(0,0);
        velocityField.x = (wheelsCurrentSpeedCm[2] + wheelsCurrentSpeedCm[1] - wheelsCurrentSpeedCm[0] - wheelsCurrentSpeedCm[3]) / 4;
        velocityField.y = (wheelsCurrentSpeedCm[2] + wheelsCurrentSpeedCm[1] + wheelsCurrentSpeedCm[0] + wheelsCurrentSpeedCm[3]) / 4;
        return velocityField;
    }

    public Vector getAcceleration() {
        Vector currentVelocity = getVelocity_FieldCS();

        Vector deltaVelocity = currentVelocity.subtract(lastVelocity);
        Vector acceleration = deltaVelocity.scale(1 / GlobalData.deltaTime);

        lastVelocity = currentVelocity;
        return acceleration;
    }
    */
    public void stop() {
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
    }

    private void drive(Vector drive, double rotation) {
        final double lfPower = drive.y + drive.x + rotation;
        final double rfPower = -drive.y + drive.x + rotation;
        final double lbPower = drive.y - drive.x + rotation;
        final double rbPower = -drive.y - drive.x + rotation;
        double highestPower = 0.8;
        final double max = Math.max(Math.abs(lfPower),
                Math.max(Math.abs(lbPower), Math.max(Math.abs(rfPower), Math.abs(rbPower))));
        if (max > 1)
            highestPower = max;
        motors[0].setPower((lfPower / highestPower));
        motors[1].setPower((rfPower / highestPower));
        motors[2].setPower((lbPower / highestPower));
        motors[3].setPower((rbPower / highestPower));
    }
    private final PID anglePID = new PID(0, 0, 0, 0, 0, 0);
    //TODO: add speed parameter and control
    public void driveToPoint(Vector vector, double wantedAngleAtArrival, double speed) {

        Vector normalizedVector = vector.unit();
        double vectorAngle = vector.getAngle();
        anglePID.setWanted(wantedAngleAtArrival);

        while (true){
            operate(normalizedVector.scale(speed), anglePID.update(Angle.wrapAnglePlusMinusPI(Gyro.getAngle())));
        }
    }
}
