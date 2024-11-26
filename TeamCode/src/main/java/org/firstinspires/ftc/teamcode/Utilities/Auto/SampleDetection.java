package org.firstinspires.ftc.teamcode.Utilities.Auto;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SampleDetection {

    ColorSensor sensor;

    public SampleDetection(HardwareMap hardwareMap) {
        sensor = hardwareMap.get(ColorSensor.class, "sensor");
    }

    public boolean red = false;
    public boolean yellow = false;
    public boolean blue = false;

    public double sensorDistance;
    public boolean sampleDetected;

    public void loopSensor() {
        sensorDistance = ((DistanceSensor) sensor).getDistance(DistanceUnit.CM);

        if (sensor.red() > 4000 && sensor.green() > 6000) {
            yellow = true;
        } else if (sensor.blue() > 3000) {
            blue = true;
        } else if (sensor.red() > 3000) {
            red = true;
        } else {
            red = false;
            yellow = false;
            blue = false;
        }

        if (sensorDistance < 1) {
            sampleDetected = true;
        } else {
            sampleDetected = false;
        }
    }
}
