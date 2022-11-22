package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.res.Gyro;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.Vector;

@TeleOp(name = "TeleOp14872")
public class TeleOp14872 extends LinearOpMode {

    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain drivetrain = new Drivetrain();

        Gyro.init(hardwareMap);
        drivetrain.init(hardwareMap);
        GlobalData.isAutonomous = false;



        waitForStart();

        while (!isStopRequested()){


            GlobalData.currentTime = timer.milliseconds();
            GlobalData.deltaTime = GlobalData.currentTime - GlobalData.lastTime;
            if(gamepad1.right_bumper) {Gyro.resetGyro();}
            drivetrain.operate(new Vector(gamepad1.left_stick_x, -gamepad1.left_stick_y), gamepad1.right_trigger - gamepad1.left_trigger);

            GlobalData.lastTime = GlobalData.currentTime;

            telemetry.addData("x value", gamepad1.left_stick_x);
            telemetry.addData("y value", gamepad1.left_stick_y);
            telemetry.addData("rotation value", gamepad1.right_trigger - gamepad1.left_trigger);
            telemetry.update();
        }
    }
}
