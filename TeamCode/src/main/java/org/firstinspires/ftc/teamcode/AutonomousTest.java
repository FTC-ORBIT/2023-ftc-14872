package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.imageprocessing.aprilTags.AprilTagDetection;
import org.firstinspires.ftc.teamcode.imageprocessing.aprilTags.ParkingSpot;
import org.firstinspires.ftc.teamcode.imageprocessing.camera.Camera;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.Claw;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.sensors.Gyro;
import org.firstinspires.ftc.teamcode.sensors.RevDistanceSensor;
import org.firstinspires.ftc.teamcode.utils.Vector;

@Autonomous(name = "Tests")
public class AutonomousTest extends LinearOpMode {
    Drivetrain drivetrain = new Drivetrain();
    RevDistanceSensor revDistanceSensor = new RevDistanceSensor();
    Elevator elevator = new Elevator();
    Claw claw = new Claw();

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain.init(this);
        Camera camera = new Camera(hardwareMap);
        Gyro.init(hardwareMap);
        revDistanceSensor.init(hardwareMap);
        elevator.init(hardwareMap);
        claw.init(hardwareMap);
        AprilTagDetection.init(camera);

        telemetry.addLine("init complete");
        telemetry.update();

        waitForStart();

        claw.operate(false);
        sleep(3000);
        elevator.operate(2);
        sleep(2000);

        Gyro.resetGyro();

        parking(AprilTagDetection.findTag(telemetry));
    }

    private ParkingSpot findParkingSpot(){
        TelemetryPacket packet = new TelemetryPacket();
        ParkingSpot parkingSpot = AprilTagDetection.findTag(telemetry);
        packet.put("parking spot", parkingSpot.name());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        return parkingSpot;
    }

    public void parking(ParkingSpot parkingSpot) {
        telemetry.addData("parking spot", parkingSpot);
        telemetry.update();
        drivetrain.driveToDirection(62, 0, 0.5);
        switch (parkingSpot) {
            case LEFT:
                drivetrain.driveToDirection(55, -90, 0.8);
                break;
            case MIDDLE:
                break;
            case RIGHT:
                drivetrain.driveToDirection(55, 90, 0.8);
                break;
        }
    }
    private void wheelTest() {
        TelemetryPacket packet = new TelemetryPacket();

        for (DcMotor motor : drivetrain.motors) {
            motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            packet.put("port" , motor.getPortNumber());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            motor.setPower(1);
            sleep(2000);
            motor.setPower(0);

            packet.put("encoder", motor.getCurrentPosition());
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

        drivetrain.operate(new Vector(0.3,0.3),0);
        sleep(2000);
        drivetrain.operate(new Vector(0.5,0),0);
        sleep(2000);
        drivetrain.operate(new Vector(0,0.5),0);
        sleep(2000);
        drivetrain.operate(new Vector(0,0),1);
        sleep(2000);
    }
}