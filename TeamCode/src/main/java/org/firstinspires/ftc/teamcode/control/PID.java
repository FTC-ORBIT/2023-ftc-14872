package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PID {
    private static final ElapsedTime timer = new ElapsedTime();
    private double kP;
    private double kI;
    private double kD;
    private double iZone;
    private double kF;

    public double wanted = 0;

    private double integral = 0;

    private double prevError = 0;
    private double prevTime = 0;
    private double prevDerivative = 0;

    public PID(double kP,double kI, double kD, double kF, double iZone) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.iZone = iZone;
    }


    public void setWanted(final double wanted) {
        this.wanted = wanted;
    }

    public double update(double current) {
        final double currentError = wanted - current;
        final double currentTime = timer.milliseconds();
        final double deltaTime = currentTime - prevTime;

        if (Math.abs(currentError) < iZone) {
            if (Math.signum(currentError) != Math.signum(prevError)) {
                integral = 0;
            } else {
                integral += currentError * deltaTime;
            }
        }

        final double derivative = deltaTime == 0 ? prevDerivative : (currentError - prevError) / deltaTime;

        prevError = currentError;
        prevTime = currentTime;
        prevDerivative = derivative;
        return kP * currentError + kI * integral + kD * derivative + kF * wanted;
    }
}