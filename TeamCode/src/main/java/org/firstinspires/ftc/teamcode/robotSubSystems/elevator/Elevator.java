package org.firstinspires.ftc.teamcode.robotSubSystems.elevator;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Elevator {
    public static DcMotor elevatorMotor;
    public static int level;
    public static void init(HardwareMap hardwareMap){
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevMotor");
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        level = 0;
        elevatorMotor.setPower(0.96);
    }

    public static void operate(int level) {
        switch (level) {
            case 0:
                goToPosition(elevatorMotor,0);
                break;
            case 1:
                goToPosition(elevatorMotor,1750);
                break;
            case 2:
                goToPosition(elevatorMotor,2865);
                break;
            case 3:
                goToPosition(elevatorMotor,4085);
                break;
            default:
                elevatorMotor.setTargetPosition(0);
                break;
        }
    }

    public static void goToPosition(DcMotor motor,int target) { /*in ticks*/ motor.setTargetPosition(target); runToPos(motor); }

    private static void runToPos(DcMotor motor) {motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);}

    public static void stop(){ elevatorMotor.setPower(0);}

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