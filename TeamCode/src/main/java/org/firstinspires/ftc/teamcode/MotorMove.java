package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
@Autonomous(name = "motor1")
public class MotorMove extends OpMode {
    DcMotor intake;
    DcMotor arm;

    @Override
    public void init() {

        intake = hardwareMap.get(DcMotor.class,"intake");
        arm = hardwareMap.get(DcMotor.class,"arm");
    }

    @Override
    public void loop() {

        intake.setPower(gamepad1.right_trigger);
        intake.setPower(-gamepad1.left_trigger);
        if (gamepad1.right_bumper) {
            arm.setPower(-0.5);
        } else {
            arm.setPower(-0.01);
        }
    }
}