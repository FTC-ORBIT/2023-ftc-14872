package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.teamcode.recourses14872.HardwarePush;

@Autonomous(name = "motor1")
public class MotorMove extends OpMode {


    @Override
    public void init() {

        HardwarePush map = new HardwarePush();
        map.init(hardwareMap);
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