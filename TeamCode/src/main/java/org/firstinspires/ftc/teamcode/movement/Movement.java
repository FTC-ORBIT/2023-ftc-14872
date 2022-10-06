package org.firstinspires.ftc.teamcode.movement;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.control.PID;
import org.firstinspires.ftc.teamcode.res.Hardware;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Timer;
import java.util.Vector;

public class Movement implements Runnable {

    static double offset = 0;
    private final Hardware hardware;
    private final OpMode opMode;

    public Movement(Hardware hardware) {
        this.hardware = hardware;
        opMode = hardware.opMode;
        Thread background = new Thread(this);
        background.start();
    }


    /**
     * does all of the field centric calculations
     *
     * @return the corrected Joystick values
     */
    public double[] fieldCentric() {
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


    public void drive(double[] values) {

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
     *
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

    public void turn(double angle, TelemetryPacket packet) {
        PID turnPID = new PID(0.5, 0, 0, 0, 0, wrap(angle));
        while (getAngle() < wrap(angle)) {
            drive(new double[]{0, 0, turnPID.update(getAngle())});
            packet.put("timer", turnPID.timer);
            packet.put("gyro", getAngle());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }

    /**
     * drives to any place orients itself according to angle and distance
     *
     * @param cm       distance to drive
     * @param angle    angle to rotate to in radians
     * @param maxPower top speed for driving
     * @param minPower minimum speed for driving
     */
    public void moveAndRotate(double cm, double angle, double maxPower, double minPower) {
        PID rotationPID = new PID(0, 0, 0, 0, 0, angle);
        PID drivePID = new PID(0, 0, 0, 0, 0, cm);


        /**
         *
         * loop:
         * calculate the error of the angle using the gyro's angle
         * float timeToTarget = cm/speed
         * rotationSpeed = calculate the speed of the rotation (angle-gyrorotation /timeToTarget)
         * rotate(rotationSpeed)
         * calls the drive function
         */
    }

    public double[] wheelVelocities;
    public ArrayList<Double> velocityVector;
    //the distance between the center of the robot and wheels
    public double robotR = 0;

    @Override
    public void run() {
        ElapsedTime timer = new ElapsedTime();
        double deltaTime;
        double currentTime;
        double prevTime = 0;
        double[] wheelsPosition;
        double[] prevWheelsPosition = {hardware.lf.getCurrentPosition(), hardware.lb.getCurrentPosition(), hardware.rf.getCurrentPosition(), hardware.rb.getCurrentPosition()};
        while (true){
            wheelsPosition = new double[]{hardware.lf.getCurrentPosition(), hardware.lb.getCurrentPosition(), hardware.rf.getCurrentPosition(), hardware.rb.getCurrentPosition()};
            currentTime = timer.seconds();
            deltaTime = currentTime - prevTime;
            for (int i = 0; i < 4; i++) {
                wheelVelocities[i] = ((wheelsPosition[i] - prevWheelsPosition[i]) * timer.seconds());
            }
            velocityVector.add((wheelVelocities[2] + wheelVelocities[1] - wheelVelocities[0] - wheelVelocities[3]) / 4 * deltaTime);
            velocityVector.add((wheelVelocities[2] + wheelVelocities[1] + wheelVelocities[0] + wheelVelocities[3]) / 4 * deltaTime);
            velocityVector.add((wheelVelocities[2] + wheelVelocities[0] - wheelVelocities[1] - wheelVelocities[3]) / 2 * deltaTime);

            prevWheelsPosition = wheelsPosition;
            prevTime = currentTime;
        }
    }
}
