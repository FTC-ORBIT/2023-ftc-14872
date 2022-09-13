package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.movement.Motors;
import org.firstinspires.ftc.teamcode.res.Hardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "TeleOp14872")
public class TeleOp14872 extends OpMode {
    OpenCvWebcam webcam10 = null;
    Hardware hardware = new Hardware(this);
    Motors motors = new Motors(hardware);

    @Override
    public void init() {
        hardware.init();
    }
    @Override
    public void loop() {
        motors.drive();
    }
}
