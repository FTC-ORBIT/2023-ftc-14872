package org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain;


import android.graphics.Point;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.teamcode.control.PID;
import org.firstinspires.ftc.teamcode.res.Gyro;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.utils.Angle;
import org.firstinspires.ftc.teamcode.utils.Vector;

public class Drivetrain {

    public final DcMotor[] motors = new DcMotor[4];
    //public Vector lastVelocity = getVelocity_FieldCS();
   
   //start point of robot at start
    private Point lastPoint = new Point(0,0);
    
   
   
    public void init(HardwareMap hardwareMap) {
        //if (GlobalData.isAutonomous) drive = new SampleMecanumDrive(hardwareMap);
        motors[0] = hardwareMap.get(DcMotor.class, "lf");
        motors[1] = hardwareMap.get(DcMotor.class, "lb");
        motors[2] = hardwareMap.get(DcMotor.class, "rf");
        motors[3] = hardwareMap.get(DcMotor.class, "rb");

        for (final DcMotor motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }
    public void operate(Vector velocity_W, double rotation) {
        final double robotAngle = Angle.wrapAnglePlusMinusPI(Math.toRadians(Gyro.getAngle()) + Math.PI/2);
        drive(velocity_W.rotate(robotAngle), -rotation);
    }



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

    public void stop() {
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
    }

    private void drive(Vector drive, double rotation) {
        final double lfPower = drive.y + drive.x + rotation;
        final double rfPower = drive.y - drive.x - rotation;
        final double lbPower = drive.y - drive.x + rotation;
        final double rbPower = drive.y + drive.x - rotation;
        final double max = Math.max(1,Math.max(Math.abs(lfPower),
                Math.max(Math.abs(lbPower), Math.max(Math.abs(rfPower), Math.abs(rbPower)))));
        motors[0].setPower((lfPower / max));
        motors[1].setPower((rfPower / max));
        motors[2].setPower((lbPower / max));
        motors[3].setPower((rbPower / max));
    }

    public void turn(double wantedAngle, double kp, double kd) {
        PID anglePID = new PID(kp, 0, kd, 0, 0);
        anglePID.setWanted(wantedAngle);
        while (Math.abs(Gyro.getAngle()) <= Math.abs(wantedAngle)) {
            operate(Vector.zero(), anglePID.update(Gyro.getAngle()));
        }
    }

    public void driveForward(double distInCM, boolean direction) {
        double startPos = avgWheelPosInCM();
        PID driveForward = new PID(0,0,0,0,0);
        driveForward.setWanted(distInCM);
        if(direction) {
            while(avgWheelPosInCM() < distInCM + startPos) {
                operate(new Vector(0, 1 / (driveForward.update(avgWheelPosInCM()))), 0);
            }
        } 
        else {
            while(avgWheelPosInCM() > distInCM + startPos) {
                operate(new Vector(0, -1 / (driveForward.update(avgWheelPosInCM()))), 0);
            }
        }
       
    }



    public double avgWheelPosInCM(){
        return Math.pow((wheelsCurrentSpeedCm[2] + wheelsCurrentSpeedCm[1] - wheelsCurrentSpeedCm[0] - wheelsCurrentSpeedCm[3]) / 4, 2) + Math.pow((wheelsCurrentSpeedCm[2] + wheelsCurrentSpeedCm[1] + wheelsCurrentSpeedCm[0] + wheelsCurrentSpeedCm[3]) / 4, 2);
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

    public void goTo(double x, double y, boolean direction) {
        double adjacent = x - lastPoint.x;
        double opposite = y - lastPoint.y;
        double distance = Math.sqrt(Math.pow(adjacent,2) + Math.pow(opposite,2));
        double angle = Math.atan2(opposite,adjacent);
        turn(angle,0,0);
        driveForward(distance,direction);
        lastPoint = new Point((int)x,(int)y);
    }
}
