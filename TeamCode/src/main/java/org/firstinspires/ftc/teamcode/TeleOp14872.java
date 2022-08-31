package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.movement.Motors;
import org.firstinspires.ftc.teamcode.res.Hardware;

@TeleOp(name = "TeleOp14872")
public class TeleOp14872 extends OpMode {

    Hardware hardware = new Hardware(this);
    Motors motors = new Motors(hardware);

    @Override
    public void init() {
        hardware.init();
    }


    @Override
    public void loop() {
        motors.drive();
    }
}