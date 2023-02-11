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

@Autonomous(name = "AutonomousLeft")
public class AutonomousLeft extends LinearOpMode {

    RevDistanceSensor revDistanceSensor = new RevDistanceSensor();
    Drivetrain drivetrain = new Drivetrain();
    Claw claw = new Claw();
    Elevator elevator = new Elevator();
    @Override

    public void runOpMode() {
        drivetrain.init(this);
        Gyro.init(hardwareMap);
        Camera camera = new Camera(hardwareMap);
        claw.init(hardwareMap);
        elevator.init(hardwareMap);
        revDistanceSensor.init(hardwareMap);
        AprilTagDetection.init(camera);
        FtcDashboard.getInstance().startCameraStream(camera.get(), 60);
        claw.operate(false);

        waitForStart();

        autonomousLeft();
        parkingDeciderLeft(AprilTagDetection.findTag(telemetry));
    }

    public void autonomousLeft() {
        claw.operate(false);
        claw.operate(false);
        this.sleep(200);
        elevator.coneStackLevel(5);
        elevator.operate(2);
        drivetrain.driveToDirection(140,0,0.8);
        elevator.operate(4);
        drivetrain.driveToDirection(15,180,0.6);
        drivetrain.driveToDirection(32,-90,0.5);
        drivetrain.driveToDirection(23,0,0.4);
        drivetrain.driveToDirection(3,180,0.2);
        elevator.coneStackLevel(5);
        claw.operate(true);
        sleep(200);

        coneCyclesLeft(1);
    }

    public void parkingDeciderLeft(ParkingSpot parkingSpot) {

        switch (parkingSpot) {
            case LEFT:
                drivetrain.driveToDirection(80, 90, 0.4);
                break;
            case MIDDLE:
                drivetrain.driveToDirection(25, 90, 0.3);
                break;
            case RIGHT:
                drivetrain.driveToDirection(25, -90, 0.4);
                break;
            default:
                parkingDeciderLeft(ParkingSpot.MIDDLE);
        }
    }

    public void coneCyclesLeft(int amountOfCycles){
        for(int i = 4; i >= amountOfCycles; i--){
            drivetrain.driveToDirection(16,180,0.4);
            drivetrain.turn(90);
            drivetrain.driveToDirection(90,90,0.8);
            claw.operate(false);
            this.sleep(200);
            elevator.operate(2);
            this.sleep(200);
            drivetrain.driveToDirection(90,-90,0.7);
            elevator.operate(4);
            drivetrain.turn(0);
            drivetrain.driveToDirection(23,0,0.4);
            drivetrain.driveToDirection(3,180,0.2);
            elevator.coneStackLevel(i);
            claw.operate(true);
        }
    }


}
