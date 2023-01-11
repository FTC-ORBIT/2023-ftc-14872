package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.Claw;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.sensors.Gyro;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.Vector;

@TeleOp(name = "TeleOp14872")
public class TeleOp14872 extends LinearOpMode {

    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        Gyro.init(hardwareMap);
        Claw claw = new Claw();
        Elevator elevator = new Elevator();
        Drivetrain drivetrain = new Drivetrain();

        Gyro.init(hardwareMap);
        drivetrain.init(hardwareMap);
        elevator.init(hardwareMap);
        claw.init(hardwareMap);
        GlobalData.isAutonomous = false;

        waitForStart();

        while (!isStopRequested()){


            GlobalData.currentTime = timer.milliseconds();
            //drivetrain.operate(new Vector(gamepad1.left_stick_x, gamepad1.left_stick_y), gamepad1.left_trigger - gamepad1.right_trigger);
            //elevator.setElevatorPower(-gamepad1.right_stick_y);
            if (gamepad1.dpad_down) {elevator.operate(0);}
            if (gamepad1.dpad_up) {elevator.operate(1);}
            if (gamepad1.dpad_left) {elevator.operate(2);}
            if (gamepad1.dpad_right) {elevator.operate(3);}
            if (gamepad1.right_stick_button) {elevator.operate(4);}
            if (gamepad1.left_bumper){claw.operate(false);
            } else if(gamepad1.right_bumper){claw.operate(true);}
            GlobalData.deltaTime = GlobalData.currentTime - GlobalData.lastTime;
            GlobalData.lastTime = GlobalData.currentTime;

            telemetry.addData("left elevator pos", elevator.elevatorMotorL.getCurrentPosition());
            telemetry.addData("right elevator pos", elevator.elevatorMotorR.getCurrentPosition());
            telemetry.addData("left elevator power", elevator.elevatorMotorL.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("right elevator power", elevator.elevatorMotorR.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Claw position: ",claw.clawPos());
            telemetry.update();
        }
    }
}
