
package org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.control.PIDF;
import org.firstinspires.ftc.teamcode.sensors.Gyro;
import org.firstinspires.ftc.teamcode.utils.Angle;
import org.firstinspires.ftc.teamcode.utils.Vector;

public class Drivetrain {

    public final DcMotorEx[] motors = new DcMotorEx[4];
    Telemetry telemetry;

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
            motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }

        this.telemetry = telemetry;
    }

    public void operate(Vector velocity_W, double rotation) {
        final double robotAngle = Angle.wrapAnglePlusMinusPI(Math.toRadians(Gyro.getAngle()) + Math.PI * 1.5 );
        drive(velocity_W.rotate(-robotAngle), rotation);
    }



    public Vector getVelocity_FieldCS() {
        Vector velocityField = new Vector(0,0);
        velocityField.x = (motors[2].getVelocity() + motors[1].getVelocity() - motors[0].getVelocity() - motors[3].getVelocity()) / 4;
        velocityField.y = (motors[2].getVelocity() + motors[1].getVelocity() + motors[0].getVelocity() + motors[3].getVelocity()) / 4;
        return velocityField;
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
        motors[0].setPower((lfPower / max));
        motors[1].setPower((rfPower / max));
        motors[2].setPower((lbPower / max));
        motors[3].setPower((rbPower / max));
    }

    private boolean isTurnRunning = false;

    public void turn(double wantedAngle, double kp, double kd) {
        PIDF anglePIDF = new PIDF(DrivetrainConstants.turnPIDCoefficients);
        anglePIDF.setWanted(wantedAngle);
        if (Gyro.getAngle() >= wantedAngle - 0.5 && Gyro.getAngle() <= wantedAngle + 0.5) {
            isTurnRunning = false;
            return;
        }
        drive(Vector.zero(), anglePIDF.update(Gyro.getAngle()));
        isTurnRunning = true;
    }
    public boolean isTurnRunning(){
        return isTurnRunning;
    }
    //Dor: STOP CHANGING MY CODE WE HAVE MECANUM!!!!

    /**
     * drives to the given angle the given distance.
     * @param distInCM the amount of cm to drive.
     * @param angle the angle to drive to.
     */
    private final boolean driveToDirectionRunning = false;
    public void driveToDirection(double distInCM, double angle) {

        double beginPosition = avgWheelPosInCM();
        Vector vector = new Vector(0, 0.4 * distInCM / Math.abs(distInCM));


        while (!(Math.abs(beginPosition + distInCM) - 2 <= avgWheelPosInCM() && Math.abs(beginPosition + distInCM) + 2 >= avgWheelPosInCM())){
            operate(vector.rotate(Math.toRadians(angle)), 0);
        }
        stop();
        //TODO: add angle control
    }

    public boolean isDriveToDirectionRunning(){return driveToDirectionRunning;}
    //TODO: maybe move function to somewhere else
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

    public double wheelRatio() {
        double sum = 0;
        for(int i = 0; i < motors.length / 2; i++) {
            sum += motors[i].getCurrentPosition();
        }
        for(int i = 2; i < motors.length; i++) {
            sum -= motors[i].getCurrentPosition();
        }
        return (sum / 2) * DrivetrainConstants.ticksToCM;
    }

    public void driveForward(double distInCM) {
        double beginPosition = avgWheelPosInCM();
        Vector vector = new Vector(0, 1);


        while(beginPosition + distInCM - 1 <= avgWheelPosInCM() + distInCM && beginPosition + distInCM + 1 >= avgWheelPosInCM() + distInCM ) {
            //TODO: add angle control
            operate(vector, 0);
        }
    }


}

