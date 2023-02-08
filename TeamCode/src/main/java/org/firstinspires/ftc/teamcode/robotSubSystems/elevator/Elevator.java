package org.firstinspires.ftc.teamcode.robotSubSystems.elevator;

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
    Claw claw = new Claw();

    public void init(HardwareMap hardwareMap){

        MotorL = (DcMotorEx) hardwareMap.get(DcMotor.class, "elevMotorL");
        MotorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        MotorL.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        MotorR = (DcMotorEx) hardwareMap.get(DcMotor.class, "elevMotorR");
        MotorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        MotorL.setPower(1);
        MotorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorR.setPower(1);

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
    public void coneInsert() {
        switch (getLevel()) {
            case 2:
                goToPosition(MotorR, MotorL, ElevatorConstants.elevatorLevelsInTicks[2] - 500);
                //claw.operate(true);
                goToPosition(MotorR, MotorL, ElevatorConstants.elevatorLevelsInTicks[2]);
                break;
            case 3:
                goToPosition(MotorR, MotorL, ElevatorConstants.elevatorLevelsInTicks[3] - 500);
                //claw.operate(true);
                goToPosition(MotorR, MotorL, ElevatorConstants.elevatorLevelsInTicks[3]);
                break;
            case 4:
                goToPosition(MotorR, MotorL, ElevatorConstants.elevatorLevelsInTicks[4] - 500);
                //claw.operate(true);
                goToPosition(MotorR, MotorL, ElevatorConstants.elevatorLevelsInTicks[4]);
            default:
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

        MotorL.setPower(power * 0.8);
        MotorR.setPower(power * 0.8);
    }

    public int getLevel(){return level;}

    public void goToPosition(DcMotor leftMotor,DcMotor rightMotor, int target) {

        leftMotor.setTargetPosition(target);
        rightMotor.setTargetPosition(target);
        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public double getPosition(){
        return (getMotorLPos() + getMotorRPos()) / 2.0;
    }

    public void stop(){
        MotorL.setPower(0);
        MotorR.setPower(0);
    }

}


// first stage 34.29 (3.06)
//second stage 59.69 (5.32)
//third stage 85.09 (7.59)
//cm for motor rotation 11.2