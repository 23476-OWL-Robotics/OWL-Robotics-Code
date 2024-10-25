package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class IntakeTest extends LinearOpMode {

    CRServo left;
    CRServo right;

    ColorSensor color;
    DistanceSensor distance;

    @Override
    public void runOpMode() {

        left = hardwareMap.get(CRServo.class, "left");
        right = hardwareMap.get(CRServo.class, "right");
        color = hardwareMap.get(ColorSensor.class, "sensor");
        distance = hardwareMap.get(DistanceSensor.class, "sensor");

        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.dpad_down) {
                left.setPower(1);
                right.setPower(-1);
            } else if (gamepad1.dpad_up) {
                left.setPower(-1);
                right.setPower(1);
            } else {
                left.setPower(0);
                right.setPower(0);
            }

            telemetry.addData("Sensor Distance", distance.getDistance(DistanceUnit.INCH));
            telemetry.addData("Color RED", color.red());
            telemetry.addData("Color GREEN", color.green());
            telemetry.addData("Color BLUE", color.blue());
            telemetry.update();
        }
    }
}
