package org.firstinspires.ftc.teamcode.robotSubSystems.elevator;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.TeleOp14872;
import org.firstinspires.ftc.teamcode.control.PIDF;

public class ElevatorConstants {

    public static final double motorRotationsPerLevel = 25.4/11.2;
    public static final int[] elevatorLevelsInTicks = {0, 70, 1815, 2910, 4090};
    //TODO: find the best values.
    public static final double maxEncoderTick = 4300;

}
