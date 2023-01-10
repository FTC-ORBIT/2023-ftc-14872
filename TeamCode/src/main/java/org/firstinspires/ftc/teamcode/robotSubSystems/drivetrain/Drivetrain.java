
package org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.teamcode.control.PIDF;
import org.firstinspires.ftc.teamcode.sensors.Gyro;
import org.firstinspires.ftc.teamcode.utils.Angle;
import org.firstinspires.ftc.teamcode.utils.Vector;

public class Drivetrain {

    public final DcMotorEx[] motors = (DcMotorEx[]) new DcMotorEx[4];

    public void init(HardwareMap hardwareMap) {
        motors[0] = hardwareMap.get(DcMotorEx.class, "lf");
        motors[1] = hardwareMap.get(DcMotorEx.class, "lb");
        motors[2] = hardwareMap.get(DcMotorEx.class, "rf");
        motors[3] = hardwareMap.get(DcMotorEx.class, "rb");

        for (final DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void operate(Vector velocity_W, double rotation) {
        final double robotAngle = Angle.wrapAnglePlusMinusPI(Math.toRadians(Gyro.getAngle()) + Math.PI/2);
        drive(velocity_W.rotate(robotAngle), -rotation);
    }



    public Vector getVelocity_FieldCS() {
        Vector velocityField = new Vector(0,0);
        velocityField.x = (motors[2].getVelocity() + motors[1].getVelocity() - motors[0].getVelocity() - motors[3].getVelocity()) / 4;
        velocityField.y = (motors[2].getVelocity() + motors[1].getVelocity() + motors[0].getVelocity() + motors[3].getVelocity()) / 4;
        return velocityField;
    }

    public void stop() {
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
    }

    /**
     * drives the robot, can drive to all direction mean while rotating
     * @param directionNPower the direction to drive and the speed to drive there.
     * @param rotation which direction to rotate and at what speed.
     */
    private void drive(Vector directionNPower, double rotation) {
        final double lfPower = directionNPower.y + directionNPower.x + rotation;
        final double rfPower = directionNPower.y - directionNPower.x - rotation;
        final double lbPower = directionNPower.y - directionNPower.x + rotation;
        final double rbPower = directionNPower.y + directionNPower.x - rotation;
        final double max = Math.max(1,Math.max(Math.abs(lfPower),
                Math.max(Math.abs(lbPower), Math.max(Math.abs(rfPower), Math.abs(rbPower)))));
        motors[0].setPower((lfPower / max));
        motors[1].setPower((rfPower / max));
        motors[2].setPower((lbPower / max));
        motors[3].setPower((rbPower / max));
    }

    public void turn(double wantedAngle, double kp, double kd) {
        PIDF anglePIDF = new PIDF(DrivetrainConstants.turnPIDCoefficients);
        anglePIDF.setWanted(wantedAngle);
        while (Math.abs(Gyro.getAngle()) <= Math.abs(wantedAngle)) {
            operate(Vector.zero(), anglePIDF.update(Gyro.getAngle()));
        }
    }

    //Dor: STOP CHANGING MY CODE WE HAVE MECANUM!!!!

    /**
     * drives to the given angle the given distance.
     * @param distInCM the amount of cm to drive.
     * @param angle the angle to drive to.
     */
    public void driveToDirection(double distInCM, double angle) {
        double beginPosition = avgWheelPosInCM();
        Vector vector = new Vector(0, 1);
        vector.rotate(angle);

        while(beginPosition + distInCM - 1 <= avgWheelPosInCM() + distInCM && beginPosition + distInCM + 1 >= avgWheelPosInCM() + distInCM ) {
            //TODO: add angle control
            operate(vector, 0);
        }
    }

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
        return ticksToCm(Math.sqrt(Math.pow((motors[2].getCurrentPosition() + motors[1].getCurrentPosition() - motors[0].getCurrentPosition() - motors[3].getCurrentPosition()) / 4, 2) + Math.pow((motors[2].getCurrentPosition() + motors[1].getCurrentPosition() + motors[0].getCurrentPosition() + motors[3].getCurrentPosition()) / 4, 2)));
    }

    //TODO: explain to dor why we need this
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

    //Dor: Do we need to use that?
    /*
    public void goTo(double x, double y, boolean direction) {
        double adjacent = x - lastPoint.x;
        double opposite = y - lastPoint.y;
        double distance = Math.sqrt(Math.pow(adjacent,2) + Math.pow(opposite,2));
        double angle = Math.atan2(opposite,adjacent);
        turn(angle,0,0);
        driveForward(distance, direction);
        lastPoint = new Point((int)x,(int)y);
    }

     */
}
