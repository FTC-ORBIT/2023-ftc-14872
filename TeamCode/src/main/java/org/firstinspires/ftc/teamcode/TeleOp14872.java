package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
        Gyro.init(hardwareMap);
        //Drivetrain.init(hardwareMap);
        Elevator.init(hardwareMap);
        GlobalData.isAutonomous = false;

        waitForStart();

        while (!isStopRequested()){
            GlobalData.currentTime = timer.milliseconds();
            //Drivetrain.operate(new Vector(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.left_trigger - gamepad1.right_trigger));
            Elevator.operate(gamepad1.dpad_left,gamepad1.dpad_down,gamepad1.dpad_right);

            GlobalData.deltaTime = GlobalData.currentTime - GlobalData.lastTime;
            GlobalData.lastTime = GlobalData.currentTime;

            telemetry.update();
        }
    }
}
