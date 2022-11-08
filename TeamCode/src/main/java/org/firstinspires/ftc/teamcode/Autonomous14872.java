package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotSubSystems.camera.Camera;
import org.firstinspires.ftc.teamcode.robotSubSystems.camera.CameraConstants;
import org.openftc.easyopencv.OpenCvCamera;

@Autonomous(name = "Autonomous14872")
public class Autonomous14872 extends LinearOpMode {
    @Override
    public void runOpMode() {
        Camera.init(hardwareMap);
        while (opModeIsActive()){
            FtcDashboard.getInstance().startCameraStream(CameraConstants.camera,60);
        }
        waitForStart();

    }
}
