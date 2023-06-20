package org.firstinspires.ftc.teamcode.robotSubSystems.claw;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

    private Servo clawServo;
    private final boolean lastState = false;
    private boolean isClawOpen;
    public void init(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(Servo.class, "clawServo");
    }

    public void operate(boolean openClaw) {

        if (openClaw) {
            clawServo.setPosition(ClawConstants.clawOpenPosition);
            isClawOpen = true;
        } else {
            clawServo.setPosition(ClawConstants.clawClosedPosition);
            isClawOpen = false;
        }
    }

    public boolean isClawOpen(){
        return isClawOpen;
    }

    public double clawPos() {return clawServo.getPosition();}
}
