
package org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.PIDF;
import org.firstinspires.ftc.teamcode.sensors.Gyro;
import org.firstinspires.ftc.teamcode.utils.Angle;
import org.firstinspires.ftc.teamcode.utils.MathFuncs;
import org.firstinspires.ftc.teamcode.utils.Vector;

public class Drivetrain {

    public final DcMotorEx[] motors = new DcMotorEx[4];
    Telemetry telemetry;
    LinearOpMode opMode;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        motors[0] = (DcMotorEx) hardwareMap.get(DcMotor.class, "lf");
        motors[1] = (DcMotorEx) hardwareMap.get(DcMotor.class, "lb");
        motors[2] = (DcMotorEx) hardwareMap.get(DcMotor.class, "rf");
        motors[3] = (DcMotorEx) hardwareMap.get(DcMotor.class, "rb");

        motors[2].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);

        for (final DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        this.telemetry = telemetry;
    }

    public void init(LinearOpMode opMode) {

        HardwareMap hardwareMap = opMode.hardwareMap;

        motors[0] = (DcMotorEx) hardwareMap.get(DcMotor.class, "lf");
        motors[1] = (DcMotorEx) hardwareMap.get(DcMotor.class, "lb");
        motors[2] = (DcMotorEx) hardwareMap.get(DcMotor.class, "rf");
        motors[3] = (DcMotorEx) hardwareMap.get(DcMotor.class, "rb");

        motors[2].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);

        for (final DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }

        this.telemetry = opMode.telemetry;
        this.opMode = opMode;
    }

    public void operate(Vector velocity_W, double rotation) {
        final double robotAngle = Angle.wrapAnglePlusMinusPI(Math.toRadians(Gyro.getAngle()) + Math.PI * 1.5);
        drive(velocity_W.rotate(-robotAngle), rotation);
    }


    /**
     * get robot velocity
     * @return velocity in wheel rotations per second
     */
    public Vector getVelocity_FieldCS() {
        Vector velocityField = new Vector(0, 0);
        velocityField.x = (motors[2].getVelocity() + motors[1].getVelocity() - motors[0].getVelocity() - motors[3].getVelocity()) / 4;
        velocityField.y = (motors[2].getVelocity() + motors[1].getVelocity() + motors[0].getVelocity() + motors[3].getVelocity()) / 4;
        return velocityField.scale(1 / DrivetrainConstants.ticksPerRev);
    }

    public void stop() {
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
    }

    /**
     * drives the robot, can drive to all direction mean while rotating
     * @param directionNPower the direction to drive and the speed to drive there.
     * @param rotation which direction to rotate and at what speed.
     */
    private void drive(Vector directionNPower, double rotation) {
        final double lfPower = directionNPower.y - directionNPower.x - rotation;
        final double rfPower = directionNPower.y + directionNPower.x + rotation;
        final double lbPower = directionNPower.y + directionNPower.x - rotation;
        final double rbPower = directionNPower.y - directionNPower.x + rotation;
        final double max = Math.max(1,Math.max(Math.abs(lfPower),
                Math.max(Math.abs(lbPower), Math.max(Math.abs(rfPower), Math.abs(rbPower)))));
        motors[0].setVelocity((lfPower / max) * DrivetrainConstants.maxSpeed * DrivetrainConstants.ticksPerRev);
        motors[1].setVelocity((rfPower / max) * DrivetrainConstants.maxSpeed * DrivetrainConstants.ticksPerRev);
        motors[2].setVelocity((lbPower / max) * DrivetrainConstants.maxSpeed * DrivetrainConstants.ticksPerRev);
        motors[3].setVelocity((rbPower / max) * DrivetrainConstants.maxSpeed * DrivetrainConstants.ticksPerRev);
    }

    TelemetryPacket packet = new TelemetryPacket();

    public void turn(double wantedAngle) {
        PIDF anglePIDF = new PIDF(DrivetrainConstants.turnPIDCoefficients);
        anglePIDF.setWanted(wantedAngle);
        double power;
        while (!(Gyro.getAngle() >= wantedAngle - 0.1 && Gyro.getAngle() <= wantedAngle + 0.1) && opMode.opModeIsActive()) {
            power = anglePIDF.update(Gyro.getAngle());
            drive(Vector.zero(), power);

            packet.put("power", power);
            packet.put("angle", Gyro.getAngle());

            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

        stop();
    }

    /**
     * drives to the given angle the given distance.
     * @param distInCM the amount of cm to drive.
     * @param angle the angle to drive to.
     */
    public void driveToDirection(double distInCM, double angle, double speed) {

        double turn;
        distInCM = Math.abs(distInCM);
        double beginPosition = Math.abs(avgWheelPosInCM());
        double velocity;
        double distanceDrove = Math.abs(avgWheelPosInCM() - beginPosition);
        Vector vector;

        PIDF angleControl = new PIDF(new PIDFCoefficients(0.01, 0, 0, 0));
        angleControl.setWanted(Gyro.getAngle());

        while (Math.abs(distInCM) >= Math.abs(distanceDrove) && opMode.opModeIsActive()){
            turn = angleControl.update(Gyro.getAngle());
            distanceDrove = Math.abs(avgWheelPosInCM() - beginPosition);

            if(5 >= Math.abs(distanceDrove)) {
                telemetry.addData("speed mode", "accelerating");
                velocity = MathFuncs.smootherStep(0,5,distanceDrove + 1.5) * (speed);

            } else if (Math.abs(distInCM) - 35 * speed <= Math.abs(distanceDrove)) {
                telemetry.addData("speed mode", "decelerating");
                velocity = (1 - MathFuncs.smootherStep(0,20, distanceDrove - (distInCM - 20) - 7 * speed)) * (speed);

            } else {
                telemetry.addData("speed mode", "normal");
                velocity = speed;
            }
            vector = new Vector(0,velocity * Math.signum(distInCM));
            operate(vector.rotate(Math.toRadians(angle)), turn);

            telemetry.addData("velocity", getVelocity_FieldCS());
            telemetry.addData("distance drove", distanceDrove);
            telemetry.addData("power", velocity * DrivetrainConstants.maxSpeed);
            telemetry.addData("turn power", turn);

        }
        telemetry.addLine("finished");

        stop();
    }

    /**
     * converts the wheels motors ticks into the wheels rotation in cm.
     * @param ticks the motor ticks.
     * @return the amount of cm the wheels travels it x amount of ticks.
     */
    public double ticksToCm(double ticks){
        return ticks * DrivetrainConstants.ticksToCM;
    }

    public double avgWheelPosInCM(){
        return Math.sqrt(Math.pow(
                ticksToCm(
                        motors[2].getCurrentPosition() + motors[1].getCurrentPosition() - motors[0].getCurrentPosition() - motors[3].getCurrentPosition()) / 4, 2)
                    + Math.pow(
                            ticksToCm(motors[2].getCurrentPosition() + motors[1].getCurrentPosition() + motors[0].getCurrentPosition() + motors[3].getCurrentPosition()) / 4, 2));
    }

}

