package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.sensors.Gyro;
import org.firstinspires.ftc.teamcode.sensors.RevDistanceSensor;

@Autonomous(name = "DistanceCheck")
public class DistanceCheck extends LinearOpMode {
    Drivetrain drivetrain = new Drivetrain();
    RevDistanceSensor revDistanceSensor = new RevDistanceSensor();

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(hardwareMap, telemetry);
        Gyro.init(hardwareMap);
        revDistanceSensor.init(hardwareMap);
        waitForStart();
        drivetrain.turn(90,this);
        while (opModeIsActive()) {
            telemetry.addData("distance: ", revDistanceSensor.findDistance());
            telemetry.update();
        }

    }

    public void travelTillDist() {
        if (revDistanceSensor.findDistance() > 35) {
            drivetrain.driveToDirection(35, 90, 0.2, this);
        } else {
            drivetrain.stop();
        }
    }
}