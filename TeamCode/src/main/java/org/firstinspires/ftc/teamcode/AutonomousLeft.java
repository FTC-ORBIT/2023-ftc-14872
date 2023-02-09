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
public class AutonomousLeft extends LinearOpMode {
    RevDistanceSensor revDistanceSensor = new RevDistanceSensor();
    Drivetrain drivetrain = new Drivetrain();
    Claw claw = new Claw();
    Elevator elevator = new Elevator();
    @Override
    public void runOpMode() {
        drivetrain.init(hardwareMap, telemetry);
        Gyro.init(hardwareMap);
        claw.init(hardwareMap);
        elevator.init(hardwareMap);
        revDistanceSensor.init(hardwareMap);

        AprilTagDetection.runAprilTagDetection(this);

        waitForStart();

        claw.operate(false);
        autonomousLeft();
        parkingDeciderLeft(AprilTagDetection.wantedParkingSpot());
    }




    public void autonomousLeft() {
        claw.operate(false);
        claw.operate(false);
        this.sleep(500);
        elevator.operate(2);
        drivetrain.driveToDirection(5, 0, 0.4, this);
        drivetrain.driveToDirection(45, -90, 0.4, this);
        drivetrain.driveToDirection(85, 0, 0.4, this);
        elevator.operate(4);
        this.sleep(100);
        travelTillDistLeft(80,70);
        this.sleep(500);
        drivetrain.driveToDirection(revDistanceSensor.findDistanceB() - 15,7,0.3,this);
        elevator.operate(5);
        this.sleep(200);
        claw.operate(true);
        this.sleep(200);
        elevator.operate(4);
        drivetrain.driveToDirection(5, 180, 0.4, this);
        elevator.operate(1);

    }

    public void parkingDeciderLeft(ParkingSpot parkingSpot) {

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
                parkingDeciderLeft(ParkingSpot.MIDDLE);
        }
    }

    public void travelTillDistLeft(double dist, int limit) {
        for (int i = 0; i < limit; i++) {
            if (revDistanceSensor.findDistanceF() <= dist) {
                drivetrain.stop();
            } else {
                drivetrain.operate(new Vector(-0.3, 0), 0);
            }
        }
    }
}
