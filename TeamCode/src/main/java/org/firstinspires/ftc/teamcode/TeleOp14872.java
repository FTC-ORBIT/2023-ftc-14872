package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.movement.Movement;
import org.firstinspires.ftc.teamcode.res.Hardware;

@TeleOp(name = "TeleOp14872")
public class TeleOp14872 extends OpMode {

    Hardware hardware = new Hardware(this);
    Movement movement = new Movement(hardware);
    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void init() {
        hardware.init();

    }

    @Override
    public void loop() {
        movement.drive(movement.fieldCentric());
    }
}