package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.recourses14872.HardwarePush;

@TeleOp(name = "MecanumMove")
public class MotorMove extends OpMode {
    HardwarePush map = new HardwarePush(this);

    @Override
    public void init() {
        map.init();

    }


    @Override
    public void loop() {
        Orientation angles = map.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        map.mecanum();
    }
}