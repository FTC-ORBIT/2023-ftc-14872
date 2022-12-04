package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
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
        Gyro.init(hardwareMap);
        Drivetrain.init(hardwareMap);
        Servo servo = hardwareMap.get(Servo.class, "servoArm");

        GlobalData.isAutonomous = false;

        waitForStart();

        while (!isStopRequested()){
            GlobalData.currentTime = timer.milliseconds();
            Drivetrain.operate(new Vector(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.left_trigger - gamepad1.right_trigger));
            if (gamepad1.dpad_up) {servo.setPosition(90);}
            if (gamepad1.dpad_down) {servo.setPosition(0);}

            GlobalData.deltaTime = GlobalData.currentTime - GlobalData.lastTime;

            GlobalData.lastTime = GlobalData.currentTime;
            telemetry.update();
        }
    }
}
