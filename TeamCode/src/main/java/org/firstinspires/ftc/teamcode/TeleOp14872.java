package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Claw;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.sensors.Gyro;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.Vector;

@TeleOp(name = "TeleOp14872")
public class TeleOp14872 extends LinearOpMode {

    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        Drivetrain drivetrain = new Drivetrain();
        Claw claw = new Claw();
        Elevator elevator = new Elevator();

        Gyro.init(hardwareMap);
        drivetrain.init(hardwareMap);
        elevator.init(hardwareMap);
        claw.init(hardwareMap);

        GlobalData.isAutonomous = false;

        waitForStart();

        while (!isStopRequested()){
            GlobalData.currentTime = timer.milliseconds();
            drivetrain.operate(new Vector(gamepad1.left_stick_x, -gamepad1.left_stick_y).rotate(90), gamepad1.left_trigger - gamepad1.right_trigger);
            if (gamepad1.dpad_down) {
                elevator.operate(0);
            }else if (gamepad1.dpad_right) {
                elevator.operate(1);
            }else if (gamepad1.dpad_up) {
                elevator.operate(2);
            }else if (gamepad1.dpad_left) {
                elevator.operate(3);
            }
            if (gamepad1.left_bumper){claw.operate(true);
            }else if (gamepad1.right_bumper){claw.operate(false);}
            GlobalData.deltaTime = GlobalData.currentTime - GlobalData.lastTime;
            GlobalData.lastTime = GlobalData.currentTime;

            telemetry.update();
        }
    }
}
