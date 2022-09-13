package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.movement.Movement;
import org.firstinspires.ftc.teamcode.res.Hardware;

@Autonomous(name = "Autonomous14872")
public class Autonomous14872 extends LinearOpMode {

    Hardware hardware = new Hardware(this);
    Movement movement = new Movement(hardware);
    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void runOpMode() throws InterruptedException {
        hardware.init();
        movement.turn(90, packet);
    }
}
