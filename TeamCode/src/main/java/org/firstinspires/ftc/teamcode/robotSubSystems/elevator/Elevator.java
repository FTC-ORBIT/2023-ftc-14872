package org.firstinspires.ftc.teamcode.robotSubSystems.elevator;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import android.widget.Switch;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Elevator {
    public static DcMotor elevatorMotor;

    public static void init(HardwareMap hardwareMap){
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevatorMotor");
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public static void operate(boolean level1, boolean level2, boolean level3,int level) {
        if(level == 1) {
            //in ticks 1645
            elevatorMotor.setTargetPosition(1750);
            elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if(level == 2) {
            //in ticks
            elevatorMotor.setTargetPosition(2865);
            elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else if(level == 3) {
            //in ticks
            elevatorMotor.setTargetPosition(4085);
            elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        else{
            elevatorMotor.setTargetPosition(0);
            elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        /*
        switch (level) {
            case 1:
                //in ticks
                elevatorMotor.setTargetPosition(1654);
                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            case 2:
                //in ticks
                elevatorMotor.setTargetPosition(2865);
                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            case 3:
                elevatorMotor.setTargetPosition(4085);
                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            //default:
                //elevatorMotor.setTargetPosition(0);
        }

         */
        elevatorMotor.setPower(0.4);
    }

    public static void stop(){
        elevatorMotor.setPower(0);
    }


    public static int getPosition() {
        return elevatorMotor.getCurrentPosition();
    }
}


// first stage 34.29 (3.06)
//second stage 59.69 (5.32)
//third stage 85.09 (7.59)
//cm for motor rotation 11.2