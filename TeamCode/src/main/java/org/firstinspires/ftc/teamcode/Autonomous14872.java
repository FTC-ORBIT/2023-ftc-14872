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

        drivetrain.driveToDirection(70, 0);


    }
}
