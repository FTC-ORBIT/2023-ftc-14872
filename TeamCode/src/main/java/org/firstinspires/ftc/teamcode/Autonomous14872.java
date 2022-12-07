package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.imageprocessing.Constants;
import org.firstinspires.ftc.teamcode.imageprocessing.Measures;
import org.firstinspires.ftc.teamcode.imageprocessing.Pipeline;
import org.firstinspires.ftc.teamcode.robotSubSystems.camera.Camera;
import org.opencv.imgproc.Imgproc;

@Autonomous(name = "Autonomous14872")
public class Autonomous14872 extends LinearOpMode {
    Telemetry telemetry;
    @Override
    public void runOpMode() {
        //Camera.init function
        Camera.init(hardwareMap);
        while (true) {
            if (Pipeline.getMat() != null) break;
        }

        waitForStart();
        //FTC dashboard init
        while (opModeIsActive()) {
            FtcDashboard.getInstance().startCameraStream(Constants.camera,60);

            telemetry.addData("Distance: ", Measures.findDistance(Pipeline.getMat(),Constants.lowHSV,Constants.highHSV));
            telemetry.update();
        }

    }
}
