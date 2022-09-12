package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PID {
    public final ElapsedTime timer = new ElapsedTime();
    private final double kP;
    private final double kI;
    private final double kD;
    private final double iZone;
    private final double kF;

    private double wanted = 0;

    private double integral = 0;

    private double prevError = 0;
    private double prevTime = 0;

    /**
     * all of the pid values
     * @param kP multiples the proportional value
     * @param kI multiples the integral value
     * @param kD multiples the differential value
     * @param kF multiples the wanted value
     * @param iZone the zone of values where i is used
     * @param wanted the wanted value
     */
    public PID(double kP,double kI, double kD, double kF, double iZone, double wanted) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.iZone = iZone;
        this.wanted = wanted;
        timer.reset();
    }

    /**
     * the setter of wanted
     * @param wanted the updated wanted value
     */
    public void setWanted(final double wanted) {
        this.wanted = wanted;
    }

    /**
     * calculates the controller output with the given parameters
     * @param current the current value
     * @return returns the controller output
     */
    public double update(double current) {
        final double currentError = wanted - current;
        final double currentTime = timer.milliseconds();
        final double deltaTime = currentTime - prevTime;

        if (Math.signum(currentError) != Math.signum(prevError)){
            integral = 0;
        }else if (Math.abs(currentError) < iZone){
            integral = integral + current * deltaTime;
        }

        double derivative = (current -  prevError) / deltaTime;

        prevError = currentError;
        prevTime = deltaTime;

        return currentError * kP + kF * wanted + derivative * kD + integral * kI;
    }

}