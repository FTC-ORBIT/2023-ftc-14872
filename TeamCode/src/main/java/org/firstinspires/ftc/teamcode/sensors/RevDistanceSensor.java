package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;

public class RevDistanceSensor {
    private DistanceSensor distanceSensorB;
    private DistanceSensor distanceSensorF;
    public void init(HardwareMap hardwareMap) {
        distanceSensorF = hardwareMap.get(DistanceSensor.class, "distanceSensorB");
        distanceSensorB = hardwareMap.get(DistanceSensor.class, "distanceSensorF");
    }
    public double findDistanceB() {
        return distanceSensorB.getDistance(DistanceUnit.CM);
    }
    public double findDistanceF() {return distanceSensorF.getDistance(DistanceUnit.CM);}
}
