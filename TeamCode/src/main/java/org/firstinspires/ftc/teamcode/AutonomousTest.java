package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.imageprocessing.aprilTags.AprilTagDetection;
import org.firstinspires.ftc.teamcode.imageprocessing.aprilTags.ParkingSpot;
import org.firstinspires.ftc.teamcode.imageprocessing.camera.Camera;
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
        drivetrain.init(this);
        Camera camera = new Camera(hardwareMap);
        Gyro.init(hardwareMap);
        revDistanceSensor.init(hardwareMap);
        elevator.init(hardwareMap);
        AprilTagDetection.init(camera);
        waitForStart();
        Gyro.resetGyro();

        telemetry.addData("parking spot", AprilTagDetection.findTag(telemetry));
        telemetry.update();

        parkingDeciderLeft(AprilTagDetection.findTag(telemetry));
    }

    public void parkingDeciderLeft(ParkingSpot parkingSpot) {
        telemetry.addData("parking spot", parkingSpot);
        telemetry.update();
        switch (parkingSpot) {
            case LEFT:
                drivetrain.driveToDirection(95, 90, 0.5);
                break;
            case MIDDLE:
                drivetrain.driveToDirection(35 , 90, 0.5);
                break;
            case RIGHT:
                drivetrain.driveToDirection(35, -90, 0.5);
                break;
            default:
                parkingDeciderLeft(ParkingSpot.MIDDLE);
        }
    }
}