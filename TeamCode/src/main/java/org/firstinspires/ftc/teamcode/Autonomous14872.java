package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.imageprocessing.aprilTags.AprilTagDetection;
import org.firstinspires.ftc.teamcode.imageprocessing.aprilTags.ParkingSpot;
import org.firstinspires.ftc.teamcode.imageprocessing.camera.Camera;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.Claw;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.sensors.ColorSensorV3;
import org.firstinspires.ftc.teamcode.sensors.Gyro;
import org.firstinspires.ftc.teamcode.sensors.RevDistanceSensor;

@Autonomous(name = "Autonomous14872")
public class Autonomous14872 extends LinearOpMode {
    RevDistanceSensor revDistanceSensor = new RevDistanceSensor();
    ColorSensorV3 colorSensorV3 = new ColorSensorV3();
    Drivetrain drivetrain = new Drivetrain();
    Claw claw = new Claw();
    Elevator elevator = new Elevator();
    TelemetryPacket packet = new TelemetryPacket();
    @Override
    public void runOpMode() {
        Camera camera = new Camera(hardwareMap);
        AprilTagDetection.init(camera);
        drivetrain.init(hardwareMap, telemetry, this);
        Gyro.init(hardwareMap);
        claw.init(hardwareMap);
        elevator.init(hardwareMap);

        FtcDashboard.getInstance().startCameraStream(camera.get(), 60);

        waitForStart();

        ParkingSpot parkingSpot = AprilTagDetection.findTag(telemetry);

    }
}
