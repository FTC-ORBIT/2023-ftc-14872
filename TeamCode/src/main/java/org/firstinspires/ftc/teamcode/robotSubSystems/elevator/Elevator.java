package org.firstinspires.ftc.teamcode.robotSubSystems.elevator;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Elevator {
    private static DcMotor elevatorMotor = null;
    private int level = 0;

    public static void init(HardwareMap hardwareMap){
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevatorMotor");
    }

    public void moveElevatorToLevel(int level){

        this.level = level;
    }

    public void stop(){

    }

    public int getLevel() {
        return level;
    }
}
