package org.firstinspires.ftc.teamcode.Utilities.TeleOp;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SampleDetection {

    ColorSensor colorSensor;
    DistanceSensor distanceSensor;

    public SampleDetection(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor");
    }

    public String getSampleColor;
    public double getDistance;

    public void getValues() {
        getDistance = distanceSensor.getDistance(DistanceUnit.INCH);

        if (colorSensor.red() > 200) {
            getSampleColor = "Red Sample";
        } else if (colorSensor.blue() > 200) {
            getSampleColor = "Blue Sample";
        } else if (colorSensor.red() > 200 && colorSensor.green() > 200) {
            getSampleColor = "Yellow Sample";
        } else {
            getSampleColor = "N/A";
        }
    }
}
