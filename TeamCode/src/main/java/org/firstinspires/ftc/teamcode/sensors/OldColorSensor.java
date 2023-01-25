package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class OldColorSensor {
    private ColorSensor colorSensor;

    public void init(HardwareMap hardwareMap) {
        // Initialize the color sensor
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        // Set the LED to enabled to indicate that the sensor is active
        colorSensor.enableLed(true);
    }

    public void loop(Telemetry telemetry) {
        // Read the RGB values from the color sensor
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();
        String color = null;
        if (red > green && red > blue){
            color = "Red";
        }
        if (green > red && green > blue){
            color = "Green";
        }
        if (blue > green && blue > red){
            color = "Blue";
        }
        // Print the RGB values to the console
        telemetry.addData("Red: ",red);
        telemetry.addData("Blue: ", blue);
        telemetry.addData("Green: ", green);
        telemetry.addData("Color sensed: ", color);
        telemetry.update();
    }
}