package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.control.PID;
import org.firstinspires.ftc.teamcode.movement.Motors;
import org.firstinspires.ftc.teamcode.res.HardwarePush;
import org.firstinspires.ftc.teamcode.res.PIDC;

@TeleOp(name = "TeleOp14872")
public class TeleOp14872 extends OpMode {

    Motors motors = new Motors(this);
    TelemetryPacket packet = new TelemetryPacket();
    PID pid = new PID(PIDC.kP,PIDC.kI,PIDC.kD,PIDC.kF, PIDC.iZone);

    double prevTime = 0;
    double prevPos = 0;
    double prevSpeed = 0;

    @Override
    public void init() {motors.init();}


    @Override
    public void loop() {

        motors.drive();

        /*pid.setWanted(PIDC.wanted);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        double pos = HardwarePush.collection.getCurrentPosition();
        double time = System.currentTimeMillis();
        double speed = prevTime == 0 ? prevSpeed : ((pos - prevPos) / (time - prevTime));

        //Updating motor speed
        HardwarePush.collection.setPower(pid.update(HardwarePush.collection.getPower()));

        //Setting previous values
        prevTime = time;
        prevSpeed = speed;
        prevPos = pos;

        //Updating telemetry data
        telemetry.addData("Motor Speed", speed);

        //Updating telemetry packet data
        packet.put("Motor Speed", speed);
        packet.put("Wanted", pid.wanted);
        packet.put("Power", Math.abs(pid.update(speed)));*/

    }
}