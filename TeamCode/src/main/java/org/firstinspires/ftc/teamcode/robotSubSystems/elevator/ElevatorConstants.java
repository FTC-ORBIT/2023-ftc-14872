package org.firstinspires.ftc.teamcode.robotSubSystems.elevator;

import org.firstinspires.ftc.teamcode.control.PID;

public class ElevatorConstants {
    public static PID setElevatorLevelPID = new PID(0,0,0,0,0);
    //TODO: find values
    public static double motorRotationsPerLevel = 25.4/11.2;
}
