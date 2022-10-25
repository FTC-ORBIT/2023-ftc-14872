package org.firstinspires.ftc.teamcode.robotSubSystems.elevator;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Elevator {
    private static DcMotor elevatorMotor = null ;

    public static void init(HardwareMap hardwareMap){
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevatorMotor");
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void operate(int level){
        double elevatorWanted = level * 10;
        ElevatorConstants.setElevatorLevelPID.setWanted(elevatorWanted);
        while (Math.abs(elevatorMotor.getCurrentPosition()) < elevatorWanted) {
            elevatorMotor.setPower(ElevatorConstants.setElevatorLevelPID.update(elevatorMotor.getCurrentPosition()));
        }

        ElevatorConstants.level = level;
    }

    public void stop(){
        elevatorMotor.setPower(0);
    }


    public int getLevel() {
        return ElevatorConstants.level;
    }
}
