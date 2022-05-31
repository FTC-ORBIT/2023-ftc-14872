package org.firstinspires.ftc.teamcode.recourses14872;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwarePush {

    public DcMotor leftDrive;
    public DcMotor rightDrive;
    public DcMotor intake;
    public DcMotor arm;

    public void init(HardwareMap map){

        leftDrive = map.get(DcMotor.class, "");
        rightDrive = map.get(DcMotor.class, "");
        intake = map.get(DcMotor.class, "intake");
        arm = map.get(DcMotor.class, "arm");

        leftDrive.setPower(0);
        rightDrive.setPower(0);
        intake.setPower(0);
        arm.setPower(0);
    }
}
