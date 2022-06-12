package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.recourses14872.HardwarePush;

@TeleOp(name = "MecanumMove")
public class MotorMove extends OpMode {
    HardwarePush map = new HardwarePush(this);
    @Override
    public void init() {
        map.init();
    }


    @Override
    public void loop() {
        map.mecanum();
        telemetry.addData("Gyro",map.getAngle() * (180/Math.PI));
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Gyro",map.getAngle() * (180/Math.PI));
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}

