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
import org.firstinspires.ftc.teamcode.utils.Vector;

//need to do autonomous to the Right side too
@Autonomous(name = "Autonomous14872Left")
public class Autonomous14872 extends LinearOpMode {
    RevDistanceSensor revDistanceSensor = new RevDistanceSensor();
    ColorSensorV3 colorSensorV3 = new ColorSensorV3();
    Drivetrain drivetrain = new Drivetrain();
    Claw claw = new Claw();
    Elevator elevator = new Elevator();
    TelemetryPacket packet = new TelemetryPacket();
    @Override
    public void runOpMode() {
        drivetrain.init(hardwareMap, telemetry);
        Gyro.init(hardwareMap);
        claw.init(hardwareMap);
        elevator.init(hardwareMap);
        revDistanceSensor.init(hardwareMap);
        //colorSensorV3.init(hardwareMap);
        //FtcDashboard.getInstance().startCameraStream(camera.get(), 60);

        AprilTagDetection.runAprilTagDetection(this);


        waitForStart();
        claw.operate(false);
        //parkingDecider(parkingSpot);
        coneLeft2();
        parkingDecider2(AprilTagDetection.wantedParkingSpot());
        while (opModeIsActive()) {
            telemetry.addData("sensor", revDistanceSensor.findDistance());
            telemetry.update();
        }
    }

    public void parkingDeciderPrime(ParkingSpot parkingSpot) {

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
                parkingDeciderPrime(ParkingSpot.MIDDLE);
        }
    }

    public void parkingDecider2(ParkingSpot parkingSpot) {

        switch (parkingSpot) {
            case LEFT:
                drivetrain.driveToDirection(22, 90, 0.4, this);
                break;
            case MIDDLE:
                drivetrain.driveToDirection(6, 90, 0.3, this);
                break;
            case RIGHT:
                drivetrain.driveToDirection(9, -90, 0.4, this);
                break;
            default:
                parkingDecider2(ParkingSpot.MIDDLE);
        }
    }

    public void coneLeft1() {
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
    public void coneLeft2() {
        claw.operate(false);
        claw.operate(false);
        this.sleep(500);
        elevator.operate(2);
        drivetrain.driveToDirection(5, 0, 0.4, this);
        drivetrain.driveToDirection(45, -90, 0.4, this);
        drivetrain.driveToDirection(85, 0, 0.4, this);
        elevator.operate(4);
        this.sleep(100);
        drivetrain.driveToDirection(revDistanceSensor.findDistance()-15, 0, 0.4, this);
        elevator.operate(5);
        claw.operate(true);
        this.sleep(200);
        elevator.operate(4);
        this.sleep(500);
        drivetrain.driveToDirection(5, 180, 0.4, this);
    }
}
