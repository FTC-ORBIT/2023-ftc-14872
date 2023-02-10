package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.imageprocessing.aprilTags.AprilTagDetection;
import org.firstinspires.ftc.teamcode.imageprocessing.aprilTags.ParkingSpot;
import org.firstinspires.ftc.teamcode.imageprocessing.camera.Camera;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.Claw;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.sensors.Gyro;
import org.firstinspires.ftc.teamcode.sensors.RevDistanceSensor;
import org.firstinspires.ftc.teamcode.utils.Vector;

public class AutonomousRight extends LinearOpMode {
    RevDistanceSensor revDistanceSensor = new RevDistanceSensor();
    Drivetrain drivetrain = new Drivetrain();
    Claw claw = new Claw();
    Elevator elevator = new Elevator();

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(this);
        Camera camera = new Camera(hardwareMap);
        Gyro.init(hardwareMap);
        claw.init(hardwareMap);
        elevator.init(hardwareMap);
        revDistanceSensor.init(hardwareMap);
        AprilTagDetection.init(camera);
        FtcDashboard.getInstance().startCameraStream(camera.get(), 60);


        waitForStart();

        claw.operate(false);
        autonomousRight();
        parkingDeciderRight(AprilTagDetection.findTag(telemetry));
    }
    public void autonomousRight() {
        claw.operate(false);
        claw.operate(false);
        this.sleep(500);
        elevator.coneStackLevel(3);
        drivetrain.driveToDirection(5, 0, 0.4);
        drivetrain.driveToDirection(55, 90, 0.4);
        drivetrain.driveToDirection(85, 0, 0.4);
        elevator.operate(4);
        this.sleep(100);
        travelTillDistRight(80,70);
        this.sleep(500);
        drivetrain.driveToDirection(revDistanceSensor.findDistanceB() - 15,7,0.3);
        elevator.operate(5);
        this.sleep(200);
        claw.operate(true);
        this.sleep(200);
        elevator.operate(4);
        drivetrain.driveToDirection(5, 180, 0.4);

    }

    public void travelTillDistRight(double dist, int limit) {
        for (int i = 0; i < limit; i++) {
            if (revDistanceSensor.findDistanceF() <= dist) {
                drivetrain.stop();
            } else {
                drivetrain.operate(new Vector(0.3, 0), 0);
            }
        }
    }

    public void parkingDeciderRight(ParkingSpot parkingSpot) {

        switch (parkingSpot) {
            case LEFT:
                drivetrain.driveToDirection(9, 90, 0.4);
                break;
            case MIDDLE:
                drivetrain.driveToDirection(6, 90, 0.3);
                break;
            case RIGHT:
                drivetrain.driveToDirection(22, -90, 0.4);
                break;
            default:
                parkingDeciderRight(ParkingSpot.MIDDLE);
        }
    }
}
