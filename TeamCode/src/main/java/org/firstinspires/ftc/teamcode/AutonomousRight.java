package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name = "AutonomousRight")
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
        AprilTagDetection.init(camera);
        claw.operate(false);

        waitForStart();

        autonomousRight();
        drivetrain.driveToDirection(15,180,0.4);
        parkingDeciderRight(AprilTagDetection.findTag(telemetry));
    }
    public void autonomousRight() {
        claw.operate(false);
        claw.operate(false);
        this.sleep(200);
        elevator.coneStackLevel(5);
        elevator.operate(2);
        drivetrain.driveToDirection(140,0,0.8);
        elevator.operate(4);
        drivetrain.driveToDirection(15,180,0.6);
        drivetrain.driveToDirection(32,90,0.5);
        drivetrain.driveToDirection(23,0,0.4);
        drivetrain.driveToDirection(3,180,0.2);
        elevator.coneStackLevel(5);
        claw.operate(true);
        sleep(200);


        coneCyclesRight(1);
    }

    public void parkingDeciderRight(ParkingSpot parkingSpot) {


        telemetry.addData("parking spot", parkingSpot);
        telemetry.update();

        switch (parkingSpot) {
            case LEFT:
                drivetrain.driveToDirection(25, 90, 0.8);
                break;
            case MIDDLE:
                drivetrain.driveToDirection(25, -90, 0.8);
                break;
            case RIGHT:
                drivetrain.driveToDirection(80, -90, 0.8);
                break;
            default:
                parkingDeciderRight(ParkingSpot.MIDDLE);
        }
    }

    public void coneCyclesRight(int amountOfCycles){
        for(int i = 4; i >= amountOfCycles + 3; i--){
            drivetrain.driveToDirection(16,180,0.4);
            drivetrain.turn(-90);
            drivetrain.driveToDirection(90,-90,0.8);
            claw.operate(false);
            this.sleep(200);
            elevator.operate(2);
            this.sleep(200);
            drivetrain.driveToDirection(90,90,0.7);
            elevator.operate(4);
            drivetrain.turn(0);
            drivetrain.driveToDirection(23,0,0.4);
            drivetrain.driveToDirection(3,180,0.2);
            elevator.coneStackLevel(i);
            claw.operate(true);
        }
    }
}
