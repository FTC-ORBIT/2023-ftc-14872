package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.imageprocessing.Sleeve;
import org.firstinspires.ftc.teamcode.imageprocessing.camera.Camera;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.Claw;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.sensors.Gyro;

@Autonomous(name = "Autonomous14872")
public class Autonomous14872 extends LinearOpMode {

    Drivetrain drivetrain = new Drivetrain();
    Claw claw = new Claw();
    Sleeve sleeve = new Sleeve();
    Elevator elevator = new Elevator();

    public static Telemetry telemetry1;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry1 = telemetry;

        drivetrain.init(hardwareMap, telemetry);
        claw.init(hardwareMap);
        Gyro.init(hardwareMap);
        Camera.init(hardwareMap);

        Gyro.resetGyro();

        waitForStart();

        claw.operate(false);

        /*while (opModeIsActive() && Sleeve.color == 0){}
        parkingDecider(Sleeve.color);*/
        drivetrain.driveToDirection(100,0, 0.4, this);

    }

    public void parkingDecider(int parkingSpot) {

        switch (parkingSpot) {
            case 1:
                drivetrain.driveToDirection(5,0, 0.4,this);
                drivetrain.driveToDirection(75, 90, 0.4, this);
                drivetrain.driveToDirection(40, 0, 0.4, this);
                break;
            case 2:
                drivetrain.driveToDirection(65, 0, 0.4, this);
                break;
            case 3:
                drivetrain.driveToDirection(5,0, 0.4,this);
                drivetrain.driveToDirection(75, -90, 0.4, this);
                drivetrain.driveToDirection(40, 0, 0.4, this);
                break;
            default:
                parkingDecider(2);
        }
    }
}
