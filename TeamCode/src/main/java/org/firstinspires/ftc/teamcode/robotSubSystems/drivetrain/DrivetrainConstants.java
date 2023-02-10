package org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
@Config
public class DrivetrainConstants {
    //TODO: find values
    public static final double trackWidth = 0;
    public static final double forward_offset = 0;
    public static final double ticksPerRev = 537.7;
    /**
     * in wheel revolutions per second
     */
    public static double maxSpeed = 4.5;
    public static final double wheelCircumferenceInCM = 9.6 * Math.PI;
    public static final double ticksToCM = wheelCircumferenceInCM / ticksPerRev;
    public static final double cmPerWheelRev = 0f;
    public static PIDFCoefficients turnPIDCoefficients = new PIDFCoefficients(0.025, 0, 0.002, 0);
}
