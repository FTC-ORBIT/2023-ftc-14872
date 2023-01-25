package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotData.GlobalData;

public class PIDF {
    private final PIDFCoefficients coefficients;
    private final double iZone = 0;

    private double wanted = 0;
    private double integral = 0;
    private double prevError = 0;

    private final double deltaTime = GlobalData.deltaTime;

    /**
     * all of the pid values
     * @param coefficients the PIDF coefficients
     */
    public PIDF(PIDFCoefficients coefficients) {
        this.coefficients = coefficients;
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
        if (Math.signum(currentError) != Math.signum(prevError)){
            integral = 0;
        }else if (Math.abs(currentError) < iZone){
            integral = integral + currentError * deltaTime;
        }

        double derivative = (currentError -  prevError) / deltaTime;

        prevError = currentError;
        return currentError * coefficients.p + coefficients.f * wanted + derivative * coefficients.d + integral * coefficients.i;
    }
}