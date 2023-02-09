package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.sensors.Gyro;
import org.firstinspires.ftc.teamcode.sensors.RevDistanceSensor;
import org.firstinspires.ftc.teamcode.utils.Vector;

@Autonomous(name = "Tests")
public class AutonomousTest extends LinearOpMode {
    Drivetrain drivetrain = new Drivetrain();
    RevDistanceSensor revDistanceSensor = new RevDistanceSensor();
    Elevator elevator = new Elevator();

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(hardwareMap, telemetry);
        Gyro.init(hardwareMap);
        revDistanceSensor.init(hardwareMap);
        elevator.init(hardwareMap);
        waitForStart();
        elevator.coneStackLevel(5);

        while (opModeIsActive()) {
            telemetry.addData("distance BACK: ", revDistanceSensor.findDistanceB());
            telemetry.addData("distance FRONT: ", revDistanceSensor.findDistanceF());
            telemetry.update();
        }

    }
}