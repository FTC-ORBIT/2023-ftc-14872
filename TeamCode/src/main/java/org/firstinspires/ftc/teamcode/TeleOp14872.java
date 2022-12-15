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
        int level = 0;
        GlobalData.isAutonomous = false;
        Servo servo = hardwareMap.get(Servo.class, "servo");
        waitForStart();

        while (!isStopRequested()){
            GlobalData.currentTime = timer.milliseconds();
            //Drivetrain.operate(new Vector(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.left_trigger - gamepad1.right_trigger));

            if (gamepad1.right_bumper) {servo.setPosition(90);}
            if (gamepad1.left_bumper) {servo.setPosition(0);}

            if (gamepad1.dpad_right) {level =1;}
            if (gamepad1.dpad_down) {level =2;}
            if (gamepad1.dpad_left) {level =3;}
            Elevator.operate(gamepad1.dpad_left,gamepad1.dpad_down,gamepad1.dpad_right,level);
            if (gamepad1.a) {Elevator.elevatorMotor.setPower(0.4);}
            if (gamepad1.b) {Elevator.elevatorMotor.setPower(0);}
            GlobalData.deltaTime = GlobalData.currentTime - GlobalData.lastTime;

            GlobalData.lastTime = GlobalData.currentTime;
            if (gamepad1.dpad_down) {telemetry.addData("somthing:", "333333333333333333333333333");}
            if (gamepad1.dpad_left) {telemetry.addData("jkbvkwbekvj", "efweivfewvfef");}
            telemetry.update();
        }
    }
}
