package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.recourses14872.HardwarePush;
import org.firstinspires.ftc.teamcode.recourses14872.PID;

@TeleOp(name = "MecanumMove")
public class MotorMove extends OpMode {
    public DcMotor motor;
    HardwarePush map = new HardwarePush(this);
    double prevTime = 0;
    double prevPos = 0;
    double prevSpeed = 0;
    double prevWnted = 0;
    @Override
    public void init() {
        map.init();
        motor = hardwareMap.get(DcMotor.class , "motor");
    }


    @Override
    public void loop() {
        if (Constants.wanted != prevWnted) {
            prevTime = 0;
            prevPos = 0;
            prevSpeed = 0;
        }
        PID pid = new PID(Constants.kP , Constants.kI , Constants.kD , Constants.kF , 0);
        pid.setWanted(Constants.wanted);
        double time = System.currentTimeMillis();
        double speed = prevTime == 0 ? prevSpeed : ((motor.getCurrentPosition() - prevPos) / (time - prevTime)) * 1000;
        telemetry.addData("motor encoder" , speed);
        prevTime = time;
        prevSpeed = speed;
        prevPos = motor.getCurrentPosition();

        map.mecanum();
        telemetry.addData("Gyro",map.getAngle() * (180/Math.PI));
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Gyro",map.getAngle() * (180/Math.PI));
        packet.put("Velocity" , speed );
        packet.put("Wanted" , Constants.wanted);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        motor.setPower(pid.update(speed));
        prevWnted = Constants.wanted;

    }
}

