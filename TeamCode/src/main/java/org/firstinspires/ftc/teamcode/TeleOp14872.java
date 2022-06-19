package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.movement.Motors;
import org.firstinspires.ftc.teamcode.rescourses.HardwarePush;

@TeleOp(name = "TeleOp14872")
public class TeleOp14872 extends OpMode {

    Motors motors = new Motors(this);

    /*double prevTime = 0;
    double prevPos = 0;
    double prevSpeed = 0;
    double prevWanted = 0;
*/
    @Override
    public void init() {
        motors.init();
    }


    @Override
    public void loop() {

        motors.fieldCentric();
        /*if (PID.wanted != prevWanted)
            prevTime = 0;
            prevPos = 0;
            prevSpeed = 0;
        }
        PID pid = new PID(PID.kP , PID.kI , PID.kD , PID.kF , 0);
        pid.setWanted(PID.wanted);
        double time = System.currentTimeMillis();
        double speed = prevTime == 0 ? prevSpeed : ((motor.getCurrentPosition() - prevPos) / (time - prevTime)) * 1000;
        telemetry.addData("motor encoder" , speed);
        prevTime = time;
        prevSpeed = speed;
        prevPos = motor.getCurrentPosition();
        motors.fieldCentric();
        telemetry.addData("Gyro", motors.getAngle() * (180/Math.PI));
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Gyro", motors.getAngle() * (180/Math.PI));
        packet.put("Velocity" , speed );
        packet.put("Wanted" , PID.wanted);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        motor.setPower(pid.update(speed));
        prevWanted = PID.wanted;*/

    }
}

