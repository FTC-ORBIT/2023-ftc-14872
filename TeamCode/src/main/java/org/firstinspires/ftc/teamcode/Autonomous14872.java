package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.res.Hardware.camera;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.imageprocessing.RodDetector;
import org.firstinspires.ftc.teamcode.imageprocessing.contours;
import org.firstinspires.ftc.teamcode.res.Hardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "Autonomous14872")
public class Autonomous14872 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //calling the function that we created
        Hardware.webcamReg();
        //waitforstart func
        waitForStart();
        //FTC dashboard to work
        while (opModeIsActive()){ FtcDashboard.getInstance().startCameraStream(camera,60); }
    }
}
