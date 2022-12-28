package org.firstinspires.ftc.teamcode.robotSubSystems.elevator;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.TeleOp14872;
import org.firstinspires.ftc.teamcode.control.PID;

public class ElevatorConstants {
    public static PID setElevatorLevelPID = new PID(0,0,0,0,0);
    public static double motorRotationsPerLevel = 25.4/11.2;
}
