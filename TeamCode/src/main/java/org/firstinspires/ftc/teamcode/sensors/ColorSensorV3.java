package org.firstinspires.ftc.teamcode.sensors;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.security.PublicKey;

public class ColorSensorV3 {
    private ColorSensor colorSensor;
    //private int colorIntInt;    //can be that in colorTelemetry

    public void init(HardwareMap hardwareMap) {
        // Initialize the color sensor
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        // Set the LED to enabled to indicate that the sensor is active
        colorSensor.enableLed(true);
    }
    private int[] getColor() {
        int[] getColor = new int[3];
        getColor[0] = colorSensor.red();
        getColor[1] = colorSensor.green();
        getColor[2] = colorSensor.blue();
        return getColor;
    }

    public int color(int red, int green, int blue) {
        int colorInInt = 0;
        if (red > green && red > blue) { colorInInt = 1; }
        if (green > red && green > blue) { colorInInt = 2; }
        if (blue > green && blue > red) { colorInInt = 3; }
        return colorInInt;
    }
    
    public void colorTelemetry(Telemetry telemetry) {
        String colorStr;
        int colorInt = color(getColor()[0],getColor()[1],getColor()[2]);
        switch (colorInt) {
            case 1:
                colorStr = "red";
            break;
            case 2:
                colorStr = "blue";
            break;
            case 3:
                colorStr = "green";
            break;
            default:
                colorStr = "didn't detect color";
            break;    
        }
        telemetry.addData("Most color detected: ", colorStr);
        telemetry.update();
    }
}
