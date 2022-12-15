package org.firstinspires.ftc.teamcode.robotSubSystems.elevator;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    private static Servo clawServo;
    public static void init(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(Servo.class, "clawServo");
    }
    public static void operate(Boolean openClawBtn,Boolean closeClawBtn) {
        if (openClawBtn) {openClaw();};
        if (closeClawBtn) {closeClaw();}
    }
    public static void openClaw() {
        clawServo.setPosition(90);
    }
    public static void closeClaw() {
        clawServo.setPosition(0);
    }
}
