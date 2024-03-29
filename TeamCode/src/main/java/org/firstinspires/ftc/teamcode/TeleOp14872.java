package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotData.RobotState;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.Claw;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.sensors.Gyro;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.utils.Vector;

@TeleOp(name = "TeleOp")
public class TeleOp14872 extends OpMode {

    private final ElapsedTime timer = new ElapsedTime();

    Drivetrain drivetrain = new Drivetrain();
    Claw claw = new Claw();
    Elevator elevator = new Elevator();

    RobotState lastState = GlobalData.robotState;

    double gamepad1LeftStickOffsetX;
    double gamepad1LeftStickOffsetY;
    double gamepad1RightStickOffsetX;
    double gamepad1RightStickOffsetY;

    @Override
    public void init() {
        Gyro.init(hardwareMap);
        drivetrain.init(hardwareMap, telemetry);
        elevator.init(hardwareMap);
        claw.init(hardwareMap);

        GlobalData.robotState = RobotState.TRAVEL;
        GlobalData.isAutonomous = false;
        //Servo servo = hardwareMap.get(Servo.class, "servoArm");


        gamepad1RightStickOffsetX = gamepad1.right_stick_x;
        gamepad1RightStickOffsetY = gamepad1.right_stick_y;
        gamepad1LeftStickOffsetX = gamepad1.left_stick_x;
        gamepad1LeftStickOffsetY = gamepad1.left_stick_y;
        lastDpadState = gamepad1.dpad_down;

        claw.operate(false);
    }

    @Override
    public void loop() {

        GlobalData.currentTime = timer.milliseconds();

        GlobalData.deltaTime = GlobalData.currentTime - GlobalData.lastTime;
        GlobalData.lastTime = GlobalData.currentTime;

        changeRobotState();

        if (gamepad1.dpad_up){
            Gyro.resetGyro();
        }

        switch (GlobalData.robotState){
            case TRAVEL:
                double elevPercent = 1.25 * (4090 - elevator.getPosition()) / 4090;
                useDrive((Math.exp(elevPercent) - Math.exp(-elevPercent)) / (Math.exp(elevPercent) + Math.exp(-elevPercent)) + 0.2);
                useClaw();
                useGoByLevel();

                break;
            case COLLECTION:

                useDrive(0.6);
                useClaw();
                elevator.setElevatorPower((-gamepad1.right_stick_y - gamepad1RightStickOffsetY));

                break;
        }

        lastState = GlobalData.robotState;

        telemetry.addData("elvEncoderR",elevator.MotorR.getCurrentPosition());
        telemetry.addData("elvEncoderL",elevator.MotorL.getCurrentPosition());
        telemetry.addData("robotState", GlobalData.robotState.toString());
        telemetry.addData("stick right x", gamepad1.right_stick_x);
        telemetry.addData("stick right y", gamepad1.right_stick_y);
        telemetry.addData("stick left x", gamepad1.left_stick_x);
        telemetry.addData("stick left y", gamepad1.left_stick_y);
        telemetry.addData("elevator level", elevator.getLevel());

    }

    private void useGoByLevel(){
        if (gamepad1.a) {
            elevator.operate(1);
        }else if (gamepad1.x) {
            elevator.operate(2);
        }else if (gamepad1.y) {
            elevator.operate(3);
        }else if (gamepad1.b) {
            elevator.operate(4);
        }
    }

    private void useDrive(double powerMultiplier){
        drivetrain.operate(new Vector(-gamepad1.left_stick_x + gamepad1LeftStickOffsetX, -(gamepad1.left_stick_y - gamepad1LeftStickOffsetY)).scale(powerMultiplier), (gamepad1.right_trigger - gamepad1.left_trigger) * powerMultiplier * 1.4);
    }

    private void useClaw(){
        if (gamepad1.left_bumper){claw.operate(true);
        }else if (gamepad1.right_bumper){claw.operate(false);}
    }

    boolean lastDpadState;
    private void changeRobotState(){
        if (GlobalData.robotState == RobotState.COLLECTION){
            if (gamepad1.b || gamepad1.a || gamepad1.y || gamepad1.x){
                GlobalData.robotState = RobotState.TRAVEL;
                return;
            }
        }
        if (gamepad1.dpad_down && !lastDpadState) {
            if (GlobalData.robotState == RobotState.TRAVEL){
                GlobalData.robotState = RobotState.COLLECTION;
            } else
                GlobalData.robotState = RobotState.TRAVEL;
        }
        lastDpadState  = gamepad1.dpad_down;
    }

}
