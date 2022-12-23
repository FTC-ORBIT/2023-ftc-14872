package org.firstinspires.ftc.teamcode.robotSubSystems.elevator;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Elevator {
    private DcMotor elevatorMotor;
    private int level;

    public void init(HardwareMap hardwareMap){
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevMotor");
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        level = 0;
        elevatorMotor.setPower(0.96);
    }

    public void operate(int level) {
        switch (level) {
            case 0:
                goToPosition(elevatorMotor,0);
                this.level = 0;
                break;
            case 1:
                goToPosition(elevatorMotor,1750);
                this.level = 1;
                break;
            case 2:
                goToPosition(elevatorMotor,2870);
                this.level = 2;
                break;
            case 3:
                goToPosition(elevatorMotor,4090);
                this.level = 3;
                break;
            default:
                elevatorMotor.setTargetPosition(0);
                this.level = 0;
                break;
        }
    }

    public void goToPosition(DcMotor motor,int target) { /*in ticks*/ motor.setTargetPosition(target); runToPos(motor); }

    private void runToPos(DcMotor motor) {motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);}

    public void stop(){ elevatorMotor.setPower(0);}

}


// first stage 34.29 (3.06)
//second stage 59.69 (5.32)
//third stage 85.09 (7.59)
//cm for motor rotation 11.2


/*
        if (level == 0){
            goToPosition(elevatorMotor,0);
        }
        if(level == 1) {
            goToPosition(elevatorMotor,1750);
        }
        else if(level == 2) {
            goToPosition(elevatorMotor,2865);
        }
        else if(level == 3) {
            goToPosition(elevatorMotor,4085);
        }
        else{
            goToPosition(elevatorMotor,0);
        }
*/