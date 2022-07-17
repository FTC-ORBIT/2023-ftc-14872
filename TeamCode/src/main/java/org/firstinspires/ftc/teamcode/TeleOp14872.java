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
    public void init() {
        motors.init();
    }


    @Override
    public void loop() {

        pid.setWanted(PIDC.wanted);
        //Sending telemetry packet to the dashboard
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        //Updating the power of the motors according to the field centric drive
        //motors.drive();
        //PID testing

        //Calculating the speed of the motor
        double pos = HardwarePush.lf.getCurrentPosition();
        double time = System.currentTimeMillis();
        double speed = prevTime == 0 ? prevSpeed : ((pos - prevPos) / (time - prevTime));

        //Updating motor speed
        HardwarePush.lf.setPower(pid.update(speed));

        //Setting previous values
        prevTime = time;
        prevSpeed = speed;
        prevPos = pos;

        //Updating telemetry data
        telemetry.addData("Motor Speed", speed);
        telemetry.addData("Motor Position", pos);
        telemetry.addData("Gyro", motors.getAngle());

        //Updating telemetry packet data
        packet.put("Motor Speed", speed);
        packet.put("Motor Position", pos);
        packet.put("Wanted", pid.wanted);
        packet.put("Gyro", motors.getAngle());
        packet.put("Power", pid.update(speed));

    }
}

