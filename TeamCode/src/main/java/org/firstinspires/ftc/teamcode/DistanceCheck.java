package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.sensors.Gyro;
import org.firstinspires.ftc.teamcode.sensors.RevDistanceSensor;
import org.firstinspires.ftc.teamcode.utils.Vector;

@Autonomous(name = "DistanceCheck")
public class DistanceCheck extends LinearOpMode {
    Drivetrain drivetrain = new Drivetrain();
    RevDistanceSensor revDistanceSensor = new RevDistanceSensor();
    Autonomous14872 autonomous14872 = new Autonomous14872();

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(hardwareMap, telemetry);
        Gyro.init(hardwareMap);
        revDistanceSensor.init(hardwareMap);
        waitForStart();
        travelTillDist(40);
        while (opModeIsActive()) {
            telemetry.addData("distance: ", revDistanceSensor.findDistance());
            telemetry.update();
        }

    }
    public void travelTillDist(double dist) {
        if (revDistanceSensor.findDistance() <= dist) {
            drivetrain.stop();
        } else {
            drivetrain.operate(new Vector(-0.2, 0), 0);
        }
    }
}