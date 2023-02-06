package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.imageprocessing.aprilTags.AprilTagDetection;
import org.firstinspires.ftc.teamcode.imageprocessing.ImgprocConstants;
import org.firstinspires.ftc.teamcode.imageprocessing.camera.Camera;
import org.firstinspires.ftc.teamcode.robotSubSystems.claw.Claw;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robotSubSystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.sensors.ColorSensorV3;
import org.firstinspires.ftc.teamcode.sensors.Gyro;

@Autonomous(name = "Autonomous14872")
public class Autonomous14872 extends LinearOpMode {

    ColorSensorV3 colorSensorV3 = new ColorSensorV3();
    Drivetrain drivetrain = new Drivetrain();
    Claw claw = new Claw();
    Elevator elevator = new Elevator();
    Camera camera = new Camera(hardwareMap);
    @Override
    public void runOpMode() {
        drivetrain.init(hardwareMap,telemetry);
        Gyro.init(hardwareMap);
        claw.init(hardwareMap);
        elevator.init(hardwareMap);
        //Camera.init function
        AprilTagDetection.runAprilTagDetection(camera.get());
        //colorSensorV3.init(hardwareMap);
        waitForStart();
        switch (AprilTagDetection.wantedParkingSpot()){
            case LEFT:
                parkingDecider(1);
            case MIDDLE:
                parkingDecider(2);
            case RIGHT:
                //3
                cone();
        }

        //FTC dashboard init
        while (opModeIsActive()) {
            FtcDashboard.getInstance().startCameraStream(ImgprocConstants.camera,60);
        }

    }
    public void parkingDecider(int parkingSpot) {

        switch (parkingSpot) {
            case 1:
                drivetrain.driveToDirection(5,0, 0.4,this);
                drivetrain.driveToDirection(60, 90, 0.6, this);
                drivetrain.driveToDirection(35, 0, 0.6, this);
                break;
            case 2:
                drivetrain.driveToDirection(65, 0, 0.6, this);
                break;
            case 3:
                drivetrain.driveToDirection(5,0, 0.4,this);
                drivetrain.driveToDirection(60, -90, 0.6, this);
                drivetrain.driveToDirection(35, 0, 0.6, this);
                break;
            default:
                parkingDecider(2);
        }
    }
    public void cone() {
        claw.operate(false);
        //drivetrain.driveToDirection(65,0,0.5,this);
        //drivetrain.driveToDirection(60,-90,0.5,this);
        drivetrain.driveToDirection(5,0, 0.4,this);
        drivetrain.driveToDirection(58, -90, 0.6, this);
        drivetrain.driveToDirection(29, 0, 0.6, this);
        elevator.operate(4);
        drivetrain.driveToDirection(23,-90,0.4,this);
        drivetrain.driveToDirection(10,0,0.4,this);
        claw.operate(true);
    }
}
