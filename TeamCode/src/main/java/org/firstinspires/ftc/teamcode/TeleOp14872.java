package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotData.RobotState;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Claw;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.sensors.Gyro;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.Vector;

@TeleOp(name = "TeleOp14872")
public class TeleOp14872 extends OpMode {

    private final ElapsedTime timer = new ElapsedTime();

    Drivetrain drivetrain = new Drivetrain();
    Claw claw = new Claw();
    Elevator elevator = new Elevator();

    RobotState lastState = GlobalData.robotState;

    @Override
    public void init() {
        Gyro.init(hardwareMap);
        drivetrain.init(hardwareMap);
        elevator.init(hardwareMap);
        claw.init(hardwareMap);

        GlobalData.isAutonomous = false;
    }

    @Override
    public void loop() {

        GlobalData.currentTime = timer.milliseconds();

        GlobalData.deltaTime = GlobalData.currentTime - GlobalData.lastTime;
        GlobalData.lastTime = GlobalData.currentTime;

        switch (GlobalData.robotState){
            case TRAVEL:
                if (GlobalData.robotState != lastState){
                    elevator.operate(1);
                }
                useDrive(1);

                break;
            case COLLECTION:
                if (GlobalData.robotState != lastState){
                    elevator.operate(0);
                }
                useDrive(0.75);
                useClaw();
                elevator.setElevatorPower(-gamepad1.right_stick_y * 0.5 + 0.2);

                break;
            case UNLOADING:
                useDrive(0.5);
                useClaw();
                useGoByLevel();

                break;
        }

        lastState = GlobalData.robotState;

        changeRobotState();

        telemetry.addData("robotState", GlobalData.robotState.toString());
        telemetry.addData("joystick x", gamepad1.right_stick_x);
        telemetry.addData("joystick y", gamepad1.right_stick_y);
        telemetry.addData("elevator level", elevator.getLevel());

        telemetry.update();
    }

    private void useGoByLevel(){
        if (gamepad1.a) {
            elevator.operate(1);
        }else if (gamepad1.x) {
            elevator.operate(2);
        }else if (gamepad1.y) {
            elevator.operate(3);
        }else if (gamepad1.b){
            elevator.operate(4);
        }
    }

    private void useDrive(double powerMultiplier){
        drivetrain.operate(new Vector(gamepad1.left_stick_x, -gamepad1.left_stick_y).scale(powerMultiplier), (gamepad1.right_trigger - gamepad1.left_trigger) * powerMultiplier);
    }

    private void useClaw(){
        if (gamepad1.left_bumper){claw.operate(true);
        }else if (gamepad1.right_bumper){claw.operate(false);}
    }

    private void changeRobotState(){
        if (gamepad1.dpad_down){
            GlobalData.robotState = RobotState.TRAVEL;

        }else if (gamepad1.dpad_right){
            GlobalData.robotState = RobotState.COLLECTION;

        }else if (gamepad1.dpad_left){
            GlobalData.robotState = RobotState.UNLOADING;

        }
    }
}
