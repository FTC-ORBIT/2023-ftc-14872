package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

public class MotorMove extends OpMode {

    private DcMotor motor;
    private HardwarePushbot robot = new HardwarePushbot();

    private HardwareMap hardwareMap;

    @Override
    public void init() {

        robot.init(hardwareMap);
        motor = robot.leftDrive;
    }

    @Override
    public void loop() {
        motor.setPower(0.2);
    }
}