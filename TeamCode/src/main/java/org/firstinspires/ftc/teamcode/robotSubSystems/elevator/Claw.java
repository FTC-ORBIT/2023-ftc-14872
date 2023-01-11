package org.firstinspires.ftc.teamcode.robotSubSystems.elevator;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

    private Servo clawServo;
    private boolean lastState = false;
    private boolean isClawOpen;
    public void init(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(Servo.class, "clawServo");
    }
    public void operate(boolean openClaw) {

        if (openClaw) {
            clawServo.setPosition(0.57);
            isClawOpen = true;
        } else {
            clawServo.setPosition(1);   
            isClawOpen = false;
        }
    }

    public boolean isClawOpen(){
        return isClawOpen;
    }

}
