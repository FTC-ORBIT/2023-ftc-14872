package org.firstinspires.ftc.teamcode.robotSubSystems.elevator;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robotSubSystems.claw.Claw;


public class Elevator {
    public DcMotorEx MotorL;
    public DcMotorEx MotorR;
    private int level;
    private int coneStackLevel;
    Claw claw = new Claw();

    public void init(HardwareMap hardwareMap){

        MotorL = (DcMotorEx) hardwareMap.get(DcMotor.class, "elevMotorL");
        MotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        MotorR = (DcMotorEx) hardwareMap.get(DcMotor.class, "elevMotorR");
        MotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorR.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        level = 0;
    }

    public int getMotorLPos(){
        return MotorL.getCurrentPosition();
    }

    public int getMotorRPos(){
        return MotorR.getCurrentPosition();
    }

    /**
     * goes from level 0 - 4 in the elevator look at ElevatorConstants to see values
     * @param level level 0 is default and 1 - 4 are not
     */
    public void operate(int level) {
        MotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorL.setPower(ElevatorConstants.elevatorSpeed);
        MotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorR.setPower(ElevatorConstants.elevatorSpeed);

        if ((MotorL.getCurrentPosition() < 0 || MotorR.getCurrentPosition() < 0) || (MotorL.getCurrentPosition() > ElevatorConstants.maxEncoderTick || MotorR.getCurrentPosition() > ElevatorConstants.maxEncoderTick)) {
            stop();
            return;
        }

        switch (level) {
            case 1:
                goToPosition(MotorL, MotorR, ElevatorConstants.elevatorLevelsInTicks[0]);
                this.level = 1;
                break;
            case 2:
                goToPosition(MotorL, MotorR, ElevatorConstants.elevatorLevelsInTicks[2]);
                this.level = 2;
                break;
            case 3:
                goToPosition(MotorL, MotorR, ElevatorConstants.elevatorLevelsInTicks[3]);
                this.level = 3;
                break;
            case 4:
                goToPosition(MotorL, MotorR, ElevatorConstants.elevatorLevelsInTicks[4]);
                this.level = 4;
                break;
            case 5:
                goToPosition(MotorL, MotorR, ElevatorConstants.elevatorLevelsInTicks[1]);
                this.level = 5;
            default:
                goToPosition(MotorL, MotorR, ElevatorConstants.elevatorLevelsInTicks[0]);
                this.level = 0;
                break;
        }
    }

    public void setElevatorPower(double power){

        MotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        MotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if ((MotorL.getCurrentPosition() < 0 || MotorR.getCurrentPosition() < 0) && power < 0 || (MotorL.getCurrentPosition() > ElevatorConstants.maxEncoderTick || MotorR.getCurrentPosition() > ElevatorConstants.maxEncoderTick) && power > 0) {
            stop();
            return;
        }

        MotorL.setPower(power);
        MotorR.setPower(power);
    }

    public int getLevel(){return level;}

    public void goToPosition(DcMotor leftMotor,DcMotor rightMotor, int target) {
        rightMotor.setTargetPosition(target);
        leftMotor.setTargetPosition(target);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public double getPosition(){
        return (getMotorLPos() + getMotorRPos()) / 2.0;
    }

    public void stop(){
        MotorL.setPower(0);
        MotorR.setPower(0);
    }

    //gets elevator to the right level according to cone stack for autonomous
    public void coneStackLevel(int wantedConeStackLevel) {
        MotorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorL.setPower(ElevatorConstants.elevatorSpeed);
        MotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorR.setPower(ElevatorConstants.elevatorSpeed);
        if ((MotorL.getCurrentPosition() < 0 || MotorR.getCurrentPosition() < 0) || (MotorL.getCurrentPosition() > ElevatorConstants.maxEncoderTick || MotorR.getCurrentPosition() > ElevatorConstants.maxEncoderTick)) {
            stop();
            return;
        }
        switch (wantedConeStackLevel) {
            case 1:
                goToPosition(MotorL, MotorR, ElevatorConstants.coneStackLevelsInTicks[0]);
                coneStackLevel = 1;
                break;
            case 2:
                goToPosition(MotorL, MotorR, ElevatorConstants.coneStackLevelsInTicks[1]);
                coneStackLevel = 2;
                break;
            case 3:
                goToPosition(MotorL, MotorR, ElevatorConstants.coneStackLevelsInTicks[2]);
                coneStackLevel = 3;
                break;
            case 4:
                goToPosition(MotorL, MotorR, ElevatorConstants.coneStackLevelsInTicks[3]);
                coneStackLevel = 4;
                break;
            case 5:
                goToPosition(MotorL, MotorR, ElevatorConstants.coneStackLevelsInTicks[4]);
                coneStackLevel = 5;
                break;

        }
    }
}


// first stage 34.29 (3.06)
//second stage 59.69 (5.32)
//third stage 85.09 (7.59)
//cm for motor rotation 11.2