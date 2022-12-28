package org.firstinspires.ftc.teamcode.robotSubSystems.elevator;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Elevator {
    private DcMotorEx elevatorMotor;
    private int level;

    public void init(HardwareMap hardwareMap){
        elevatorMotor = hardwareMap.get(DcMotorEx.class, "elevMotor");
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ElevatorConstants.pidfCoefficients);
        level = 0;
    }

    public int getMotorPos(){
        return elevatorMotor.getCurrentPosition();
    }

    /**
     * goes from level 0 - 4 in the elevator look at ElevatorConstants to see values
     * @param level level 0 is default and 1 - 4 are not
     */
    public void operate(int level) {
        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorMotor.setPower(1);

        switch (level) {
            case 1:
                goToPosition(elevatorMotor, ElevatorConstants.elevatorLevelsInTicks[1]);
                this.level = 1;
                break;
            case 2:
                goToPosition(elevatorMotor,ElevatorConstants.elevatorLevelsInTicks[2]);
                this.level = 2;
                break;
            case 3:
                goToPosition(elevatorMotor,ElevatorConstants.elevatorLevelsInTicks[3]);
                this.level = 3;
                break;
            case 4:
                goToPosition(elevatorMotor,ElevatorConstants.elevatorLevelsInTicks[4]);
                this.level = 4;
                break;
            default:
                goToPosition(elevatorMotor, ElevatorConstants.elevatorLevelsInTicks[0]);
                this.level = 0;
                break;
        }
    }

    public void setElevatorPower(double power){
        elevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        elevatorMotor.setPower(power);
    }

    public int getLevel(){return level;}

    public void goToPosition(DcMotor motor,int target) { /*in ticks*/ motor.setTargetPosition(target); motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);}

    public void stop(){ elevatorMotor.setPower(0);}

}


// first stage 34.29 (3.06)
//second stage 59.69 (5.32)
//third stage 85.09 (7.59)
//cm for motor rotation 11.2