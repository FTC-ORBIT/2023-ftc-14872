package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;

public class PIDF {
    private double time = System.currentTimeMillis();
    private double kP;
    private double kI;
    private double kD;
    private double iZone;
    private double kF;

    private double wanted = 0;

    private double integral = 0;

    private double prevError = 0;
    private double prevTime = System.currentTimeMillis();

    /**
     * all of the pid values
     * @param coefficients the PIDF coefficients
     */
    public PIDF(PIDFCoefficients coefficients) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.iZone = iZone;
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
        double currentTime = System.currentTimeMillis();
        double deltaTime = 200 * GlobalData.epsilon;
        if (Math.signum(currentError) != Math.signum(prevError)){
            integral = 0;
        }else if (Math.abs(currentError) < iZone){
            integral = integral + currentError * deltaTime;
        }

        double derivative = (currentError -  prevError) / deltaTime;

        prevError = currentError;
        prevTime = currentTime;
        return currentError * kP + kF * wanted + derivative * kD + integral * kI;
    }

}