package org.firstinspires.ftc.teamcode.robotSubSystems.elevator;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Elevator {
    private static DcMotor elevatorMotor;

    public void init(HardwareMap hardwareMap){
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevatorMotor");
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void operate(int level){
        if(level == 1) {
            //in ticks
            elevatorMotor.setTargetPosition(1645);
        }
        else if(level == 2) {
            //in ticks
            elevatorMotor.setTargetPosition(2865);
        }
        else if(level == 3) {
            //in ticks
            elevatorMotor.setTargetPosition(4085);
        }
        else{
            elevatorMotor.setTargetPosition(0);
        }
    }

    public void stop(){
        elevatorMotor.setPower(0);
    }


    public int getPosition() {
        return elevatorMotor.getCurrentPosition();
    }
}


// first stage 34.29 (3.06)
//second stage 59.69 (5.32)
//third stage 85.09 (7.59)
//cm for motor rotation 11.2