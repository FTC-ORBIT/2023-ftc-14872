package org.firstinspires.ftc.teamcode.robotSubSystems.elevator;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.control.PIDF;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.DrivetrainConstants;


public class Elevator {
    public DcMotorEx elevatorMotorL;
    public DcMotorEx elevatorMotorR;
    private int level;

    public void init(HardwareMap hardwareMap){

        elevatorMotorL = (DcMotorEx) hardwareMap.get(DcMotor.class, "elevMotorL");
        elevatorMotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorMotorL.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotorL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ElevatorConstants.pidfCoefficients);

        elevatorMotorR = (DcMotorEx) hardwareMap.get(DcMotor.class, "elevMotorR");
        elevatorMotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotorR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ElevatorConstants.pidfCoefficients);

        level = 0;
    }

    public int getMotorLPos(){
        return elevatorMotorL.getCurrentPosition();
    }

    public int getMotorRPos(){
        return elevatorMotorR.getCurrentPosition();
    }

    /**
     * goes from level 0 - 4 in the elevator look at ElevatorConstants to see values
     * @param level level 0 is default and 1 - 4 are not
     */
    public void operate(int level) {
        elevatorMotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorMotorL.setPower(1);
        elevatorMotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorMotorR.setPower(1);

        switch (level) {
            case 1:
                goToPosition(elevatorMotorL, elevatorMotorR, ElevatorConstants.elevatorLevelsInTicks[1]);
                this.level = 1;
                break;
            case 2:
                goToPosition(elevatorMotorL, elevatorMotorR, ElevatorConstants.elevatorLevelsInTicks[2]);
                this.level = 2;
                break;
            case 3:
                goToPosition(elevatorMotorL, elevatorMotorR, ElevatorConstants.elevatorLevelsInTicks[3]);
                this.level = 3;
                break;
            case 4:
                goToPosition(elevatorMotorL, elevatorMotorR, ElevatorConstants.elevatorLevelsInTicks[4]);
                this.level = 4;
                break;
            default:
                goToPosition(elevatorMotorL, elevatorMotorR, ElevatorConstants.elevatorLevelsInTicks[0]);
                this.level = 0;
                break;
        }
    }

    public void setElevatorPower(double power){

        if (elevatorMotorL.getCurrentPosition() < 0 || elevatorMotorR.getCurrentPosition() < 0 || elevatorMotorL.getCurrentPosition() > 1000 || elevatorMotorR.getCurrentPosition() > 1000) {
            return;
        }

        elevatorMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevatorMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (Math.abs(power) > 0.1) {
            elevatorMotorL.setPower(0.1);
            elevatorMotorR.setPower(0.1);
        } else {
            elevatorMotorL.setPower(power);
            elevatorMotorR.setPower(power);
        }

    }

    public int getLevel(){return level;}

    public void goToPosition(DcMotor leftMotor,DcMotor rightMotor, int target) {

        leftMotor.setTargetPosition(target);
        rightMotor.setTargetPosition(target);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void stop(){
        elevatorMotorL.setPower(0);
        elevatorMotorR.setPower(0);
    }

}


// first stage 34.29 (3.06)
//second stage 59.69 (5.32)
//third stage 85.09 (7.59)
//cm for motor rotation 11.2