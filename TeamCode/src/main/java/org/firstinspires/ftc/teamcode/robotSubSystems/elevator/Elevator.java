package org.firstinspires.ftc.teamcode.robotSubSystems.elevator;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Elevator {
    private static DcMotor elevMotor;

    public static void init(HardwareMap hardwareMap){
        elevMotor = hardwareMap.get(DcMotor.class, "elevMotor");
        elevMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        elevMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public static void operate(boolean level1Btn, boolean level2Btn, boolean level3Btn) {

        if( level(level1Btn,level2Btn,level3Btn ) == 1) {
            goToPosition(elevMotor,1645);
            Claw.openClaw();
            off();
        }

        else if( level(level1Btn,level2Btn,level3Btn )  == 2) {
            goToPosition(elevMotor,2865);
            Claw.openClaw();
            off();
        }

        else if( level(level1Btn,level2Btn,level3Btn )  == 3) {
            goToPosition(elevMotor,4085);
            Claw.openClaw();
            off();
        }

        else{ off(); }

        elevMotor.setPower(0.55);
    }
    public static int level(boolean level1Btn, boolean level2Btn, boolean level3Btn) {
        int level = 0;
        if (level1Btn) {level = 1;}
        if (level2Btn) {level = 2;}
        if (level3Btn) {level = 3;}
        return level;
    }

    public static void off() {elevMotor.setTargetPosition(0); runToPos(elevMotor);}

    public static void goToPosition(DcMotor motor,int target) {/*in ticks*/ motor.setTargetPosition(target); runToPos(elevMotor);}

    private static void runToPos(DcMotor motor) {motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);}

}


// first stage 34.29 (3.06)
//second stage 59.69 (5.32)
//third stage 85.09 (7.59)
//cm for motor rotation 11.2

/*
        switch (level(level1Btn,level2Btn,level3Btn )) {
            case 1:
                goToPosition(elevMotor,1645);
                Claw.openClaw();
                off();
                break;
            case 2:
                 goToPosition(elevMotor,2865);
                 Claw.openClaw();
                 off();
                break;
            case 3:
                goToPosition(elevMotor,4085);
                Claw.openClaw();
                off();
                break;
            default:
                off();
        }
        elevMotor.setPower(0.55);
 */