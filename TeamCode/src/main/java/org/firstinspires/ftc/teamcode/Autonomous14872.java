package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.imageprocessing.aprilTags.AprilTagDetection;
import org.firstinspires.ftc.teamcode.imageprocessing.aprilTags.ParkingSpot;
import org.firstinspires.ftc.teamcode.imageprocessing.camera.Camera;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.Claw;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.sensors.ColorSensorV3;
import org.firstinspires.ftc.teamcode.sensors.Gyro;

@Autonomous(name = "Autonomous14872")
public class Autonomous14872 extends LinearOpMode {

    ColorSensorV3 colorSensorV3 = new ColorSensorV3();
    Drivetrain drivetrain = new Drivetrain();
    Claw claw = new Claw();
    Elevator elevator = new Elevator();
    TelemetryPacket packet = new TelemetryPacket();
    @Override
    public void runOpMode() {

        Camera camera = new Camera(hardwareMap);
        drivetrain.init(hardwareMap, telemetry);
        Gyro.init(hardwareMap);
        claw.init(hardwareMap);
        elevator.init(hardwareMap);
        //colorSensorV3.init(hardwareMap);
        FtcDashboard.getInstance().startCameraStream(camera.get(), 60);

        telemetry.addData("tag", AprilTagDetection.findTag(camera.get(), telemetry));
        packet.put("tag", AprilTagDetection.findTag(camera.get(), telemetry));
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        telemetry.update();
        ParkingSpot parkingSpot = AprilTagDetection.findTag(camera.get(), telemetry);

        waitForStart();
        parkingDecider(parkingSpot);
            //FTC dashboard init
        /*while (AprilTagDetection.findTag(camera.get(), telemetry) == ParkingSpot.NONE && opModeIsActive()){
            telemetry.addData("tag", AprilTagDetection.findTag(camera.get(), telemetry));
            packet.put("tag", AprilTagDetection.findTag(camera.get(), telemetry));
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            telemetry.update();
        }

         */

    }

    public void parkingDecider(ParkingSpot parkingSpot) {

        switch (parkingSpot) {
            case LEFT:
                drivetrain.driveToDirection(5, 0, 0.4, this);
                drivetrain.driveToDirection(60, 90, 0.6, this);
                drivetrain.driveToDirection(35, 0, 0.6, this);
                break;
            case MIDDLE:
                drivetrain.driveToDirection(65, 0, 0.6, this);
                break;
            case RIGHT:
                drivetrain.driveToDirection(5, 0, 0.4, this);
                drivetrain.driveToDirection(60, -90, 0.6, this);
                drivetrain.driveToDirection(35, 0, 0.6, this);
                break;
            default:
                parkingDecider(ParkingSpot.MIDDLE);
        }
    }

    public void cone() {
        claw.operate(false);
        //drivetrain.driveToDirection(65,0,0.5,this);
        //drivetrain.driveToDirection(60,-90,0.5,this);
        drivetrain.driveToDirection(5, 0, 0.4, this);
        drivetrain.driveToDirection(58, -90, 0.6, this);
        drivetrain.driveToDirection(29, 0, 0.6, this);
        elevator.operate(4);
        drivetrain.driveToDirection(23, -90, 0.4, this);
        drivetrain.driveToDirection(10, 0, 0.4, this);
        claw.operate(true);
    }
}
