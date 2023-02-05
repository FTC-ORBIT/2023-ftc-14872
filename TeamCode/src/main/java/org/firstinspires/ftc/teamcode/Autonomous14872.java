package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.aprilTags.AprilTagDetection;
import org.firstinspires.ftc.teamcode.imageprocessing.Constants;
import org.firstinspires.ftc.teamcode.imageprocessing.Pipeline;
import org.firstinspires.ftc.teamcode.imageprocessing.Sleeve;
import org.firstinspires.ftc.teamcode.robotSubSystems.camera.Camera;
import org.firstinspires.ftc.teamcode.sensors.ColorSensorV3;

@Autonomous(name = "Autonomous14872")
public class Autonomous14872 extends LinearOpMode {
    ColorSensorV3 colorSensorV3 = new ColorSensorV3();
    Sleeve sleeve = new Sleeve();
    @Override
    public void runOpMode() {
        //Camera.init function
        AprilTagDetection.runAprilTagDetection(this);
        //colorSensorV3.init(hardwareMap);
        waitForStart();
        switch (AprilTagDetection.wantedParkingSpot()){
            case LEFT:

            case RIGHT:

            case MIDDLE:

        }

        //FTC dashboard init
        while (opModeIsActive()) {
            FtcDashboard.getInstance().startCameraStream(Constants.camera,60);
        }

    }
}
