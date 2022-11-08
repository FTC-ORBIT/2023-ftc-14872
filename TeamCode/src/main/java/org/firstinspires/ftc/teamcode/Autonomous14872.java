package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.imageprocessing.Constants;
import org.firstinspires.ftc.teamcode.robotSubSystems.camera.Camera;

@Autonomous(name = "Autonomous14872")
public class Autonomous14872 extends LinearOpMode {
    @Override
    public void runOpMode() {
        //Camera.init function
        Camera.init(hardwareMap);
        waitForStart();
        //FTC dashboard init
        while (opModeIsActive()) { FtcDashboard.getInstance().startCameraStream(Constants.camera,60); }

    }
}
