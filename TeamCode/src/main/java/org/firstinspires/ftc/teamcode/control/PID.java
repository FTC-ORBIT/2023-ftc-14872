package org.firstinspires.ftc.teamcode.control;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    private ElapsedTime timer;
    public double kP = 0;
    public double kI = 0;
    public double kD = 0;
    public double kF = 0;
    public double iZone = 0;

    public double wanted = 0;

    private double integral = 0;

    private double prevError;
    private double prevTime;
    private double prevDerivative;

    public PID(final double kP, final double kI, final double kD, final double kF, final double iZone) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.iZone = iZone;
    }

    public void start(final double wanted) {
        this.wanted = wanted;
        timer = new ElapsedTime();
        prevDerivative = 0;
        prevError = 0;
        prevTime = 0;
        integral = 0;
    }

    public double update(final double current) {
        final double currentError = wanted - current;
        final double deltaTime = timer.milliseconds() - prevTime;

        if (Math.abs(currentError) < iZone) {
            if (Math.signum(currentError) != Math.signum(prevError)) {
                integral = 0;
            } else {
                integral += currentError * deltaTime;
            }
        }

        final double derivative = deltaTime == 0 ? prevDerivative : (currentError - prevError) / deltaTime;

        prevError = currentError;
        prevTime = timer.milliseconds();
        prevDerivative = derivative;

        return kP * currentError + kI * integral + kD * derivative + kF * wanted;
    }
}