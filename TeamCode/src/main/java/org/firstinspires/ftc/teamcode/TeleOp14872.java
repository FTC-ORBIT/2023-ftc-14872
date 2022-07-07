package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.control.PID;
import org.firstinspires.ftc.teamcode.movement.Motors;
import org.firstinspires.ftc.teamcode.rescourses.HardwarePush;
import org.firstinspires.ftc.teamcode.rescourses.MotorConstants;

@TeleOp(name = "TeleOp14872")
public class TeleOp14872 extends OpMode {

    Motors motors = new Motors(this);
    TelemetryPacket packet = new TelemetryPacket();

    double prevTime = 0;
    double prevPos = 0;
    double prevSpeed = 0;

    @Override
    public void init() {
        motors.init();
    }


    @Override
    public void loop() {

        //Sending telemetry packet to the dashboard
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        //Updating the power of the motors according to the field centric drive
        motors.fieldCentric();

        //PID testing
        PID pid = new PID(MotorConstants.kP,MotorConstants.kI,MotorConstants.kD,MotorConstants.kF,MotorConstants.iZone);
        pid.setWanted(MotorConstants.wanted);

        //Calculating the speed of the motor
        double pos = motors.motor.getCurrentPosition();
        double time = System.currentTimeMillis();
        double speed = time - prevTime == 0 || prevTime == 0 ? prevSpeed : ((pos - prevPos) / (time - prevTime)) * 1000;

        //Updating motor speed
        motors.motor.setPower(pid.update(speed));

        //Setting previous values
        prevTime = time;
        prevSpeed = speed;
        prevPos = pos;

        //Updating telemetry data
        telemetry.addData("Motor Speed", speed);
        telemetry.addData("Motor Position", pos);
        telemetry.addData("Gyro", motors.getAngle());
        telemetry.addData("Time", time);

        //Updating telemetry packet data
        packet.put("Motor Speed", speed );
        packet.put("Motor Position", pos);
        packet.put("Wanted", MotorConstants.wanted);
        packet.put("Gyro", motors.getAngle());
        packet.put("Time", time);

    }
}

