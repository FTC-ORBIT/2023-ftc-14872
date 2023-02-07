package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;

public class RevDistanceSensor {
    Drivetrain drivetrain = new Drivetrain();
    private DistanceSensor distanceSensor;
    public void init(HardwareMap hardwareMap) {
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
    }
    public double findDistance() {
        return distanceSensor.getDistance(DistanceUnit.CM);
    }
}
