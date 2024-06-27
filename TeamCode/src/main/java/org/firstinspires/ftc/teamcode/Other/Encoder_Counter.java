package org.firstinspires.ftc.teamcode.Other;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Encoder_Counter extends LinearOpMode {

    DcMotor motor;

    public void runOpMode() {

        motor = hardwareMap.get(DcMotor.class, "motor");

        //ToDo: To Get rotPerTick
        // divide the number of rotations by the encoder position
        // rotations / encoderPosition

        //ToDo: To Get inPerTick
        // divide the number on inches traveled by the encoder position
        // inches / encoderPosition

        waitForStart();
        while (opModeIsActive()) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            double encoderPosition = motor.getCurrentPosition();

            telemetry.addData("Encoder Position:", encoderPosition);
            telemetry.update();
        }
    }
}
