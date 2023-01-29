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

        waitForStart();

        claw.operate(false);

        while (opModeIsActive() && Sleeve.color == 0){}
        int i = Sleeve.color;
        /*drivetrain.driveToDirection(60, 0);
        drivetrain.driveToDirection(15, 90);
        elevator.operate(3);
        claw.operate(true);
        elevator.operate(2);
        drivetrain.driveToDirection(15, -90);
        drivetrain.driveToDirection(60, 180);*/

        telemetry.addData("i", i);
        telemetry.update();
        parkingDecider(i);



    }

    public void parkingDecider(int parkingSpot) {

        switch (parkingSpot) {
            case 1:
                drivetrain.driveToDirection(70, 90);
                drivetrain.driveToDirection(65, 0);
                break;
            case 2:
                drivetrain.driveToDirection(65, 0);
                break;
            case 3:
                drivetrain.driveToDirection(70, -90);
                drivetrain.driveToDirection(65, 0);
                break;
            default:
                parkingDecider(2);
        }
    }
}
