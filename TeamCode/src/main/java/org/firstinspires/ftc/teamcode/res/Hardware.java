package org.firstinspires.ftc.teamcode.res;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.imageprocessing.contours;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class Hardware {

    public DcMotor lf;
    public DcMotor rf;
    public DcMotor lb;
    public DcMotor rb;
    public DcMotor collection;
    public final OpMode opMode;
    public BNO055IMU imu;

    public Hardware(final OpMode opMode) {this.opMode = opMode;}


    /**
     * Initializes the hardware
     */
    public void init() {

        collection = opMode.hardwareMap.get(DcMotor.class, "collection");
        collection.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        lf = opMode.hardwareMap.get(DcMotor.class, "lf");
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rf = opMode.hardwareMap.get(DcMotor.class, "rf");
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lb = opMode.hardwareMap.get(DcMotor.class, "lb");
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rb = opMode.hardwareMap.get(DcMotor.class, "rb");
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lb.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = opMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }
    public static OpenCvCamera camera;
    public static void webcamReg() {
        //initiates the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        //gets the camera name from the int in the driver station
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "camera");
        //to get the webcam view
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        //creates an object to the telemetry from the file rod detector
        //RodDetector detector = new RodDetector(telemetry);
        contours contours = new contours(telemetry);
        //sets the pipeline
        //camera.setPipeline(detector);
        camera.setPipeline(contours);
        //opening the camera
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            //live streaming
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
                //This will be called if the camera could not be opened
            }
        });
    }
}