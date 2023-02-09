package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.imageprocessing.aprilTags.AprilTagDetection;
import org.firstinspires.ftc.teamcode.imageprocessing.aprilTags.ParkingSpot;
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
        drivetrain.init(hardwareMap, telemetry);
        Gyro.init(hardwareMap);
        claw.init(hardwareMap);
        elevator.init(hardwareMap);
        revDistanceSensor.init(hardwareMap);

        AprilTagDetection.runAprilTagDetection(this);

        waitForStart();

        claw.operate(false);
        autonomousRight();
        parkingDeciderRight(AprilTagDetection.wantedParkingSpot());
    }
    public void autonomousRight() {
        claw.operate(false);
        claw.operate(false);
        this.sleep(500);
        elevator.operate(2);
        drivetrain.driveToDirection(5, 0, 0.4, this);
        drivetrain.driveToDirection(45, -90, 0.4, this);
        drivetrain.driveToDirection(85, 0, 0.4, this);
        elevator.operate(4);
        this.sleep(100);
        travelTillDistRight(80,70);
        this.sleep(500);
        drivetrain.driveToDirection(revDistanceSensor.findDistanceB() - 15,7,0.3,this);
        elevator.operate(5);
        this.sleep(200);
        claw.operate(true);
        this.sleep(200);
        elevator.operate(4);
        drivetrain.driveToDirection(5, 180, 0.4, this);

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
                drivetrain.driveToDirection(22, 90, 0.4, this);
                break;
            case MIDDLE:
                drivetrain.driveToDirection(6, 90, 0.3, this);
                break;
            case RIGHT:
                drivetrain.driveToDirection(9, -90, 0.4, this);
                break;
            default:
                parkingDeciderRight(ParkingSpot.MIDDLE);
        }
    }
}
