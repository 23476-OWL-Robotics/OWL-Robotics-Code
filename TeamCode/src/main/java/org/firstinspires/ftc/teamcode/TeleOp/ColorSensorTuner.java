package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.Util.ArtifactType;

@Disabled
@Configurable
@TeleOp(name = "ColorSensorTuner", group = "Tuning")
public class ColorSensorTuner extends OpMode {

    TelemetryManager telemetryM;

    ColorSensor sensor1;
    ColorSensor sensor2;
    ColorSensor sensor3;

    ArtifactType artifact;

    @IgnoreConfigurable
    int TotalRed;
    int TotalGreen;
    int TotalBlue;

    public static int RedLimit = 280;
    public static int GreenLimit = 600;
    public static int BlueLimit = 700;


    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        sensor1 = hardwareMap.get(RevColorSensorV3.class, "transferSensor1");
        sensor2 = hardwareMap.get(RevColorSensorV3.class, "transferSensor2");
        sensor3 = hardwareMap.get(RevColorSensorV3.class, "transferSensor3");

        artifact = ArtifactType.EMPTY;
    }

    @Override
    public void loop() {
        TotalRed = sensor1.red() + sensor2.red() + sensor3.red();
        TotalGreen = sensor1.green() + sensor2.green() + sensor3.green();
        TotalBlue = sensor1.blue() + sensor2.blue() + sensor3.blue();

        if (TotalRed > RedLimit  && TotalGreen > GreenLimit && TotalBlue > BlueLimit) {
            if (TotalBlue > TotalGreen) {
                artifact = ArtifactType.PURPLE;
            } else if (TotalGreen > TotalBlue) {
                artifact = ArtifactType.GREEN;
            } else {
                artifact = ArtifactType.EMPTY;
            }
        } else {
            artifact = ArtifactType.EMPTY;
        }

        telemetry.addLine("----Artifact----");
        telemetry.addData("Type", artifact);
        telemetry.addLine("----Total Values----");
        telemetry.addData("r", TotalRed);
        telemetry.addData("g", TotalGreen);
        telemetry.addData("b", TotalBlue);
        telemetry.addLine("----Sensor1----");
        telemetry.addData("r:", sensor1.red());
        telemetry.addData("g:", sensor1.green());
        telemetry.addData("b:", sensor1.blue());
        telemetry.addLine("----Sensor2----");
        telemetry.addData("r:", sensor2.red());
        telemetry.addData("g:", sensor2.green());
        telemetry.addData("b:", sensor2.blue());
        telemetry.addLine("----Sensor3----");
        telemetry.addData("r:", sensor3.red());
        telemetry.addData("g:", sensor3.green());
        telemetry.addData("b:", sensor3.blue());

        telemetry.update();
        telemetryM.update(telemetry);
    }
}
