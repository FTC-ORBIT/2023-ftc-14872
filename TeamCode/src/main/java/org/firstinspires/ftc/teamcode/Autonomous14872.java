package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.robotSubSystems.claw.Claw;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.sensors.Gyro;
import org.firstinspires.ftc.teamcode.utils.Vector;

@Autonomous(name = "Autonomous14872")
public class Autonomous14872 extends LinearOpMode {

    Drivetrain drivetrain = new Drivetrain();
    Claw claw = new Claw();

    @Override
    public void runOpMode() throws InterruptedException {

        drivetrain.init(hardwareMap, telemetry);
        claw.init(hardwareMap);
        Gyro.init(hardwareMap);

        waitForStart();

        claw.operate(false);



        parkingDecider(2);


    }

    public void parkingDecider(int parkingSpot) {
        switch (parkingSpot) {
            case 1:
                drivetrain.driveToDirection(60, 0);
                drivetrain.driveToDirection(30, -90);
                break;
            case 2:
                drivetrain.driveToDirection(65, 0);
                break;
            case 3:
                drivetrain.driveToDirection(60, 0);
                drivetrain.driveToDirection(30, 90);
                break;
        }
    }
}
