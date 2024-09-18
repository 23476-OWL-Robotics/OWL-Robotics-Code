package org.firstinspires.ftc.teamcode.Utilities.PIDF_Controller;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class EncoderCounter extends LinearOpMode {

    DcMotorEx motor;

    @Override
    public void runOpMode() {

        motor = hardwareMap.get(DcMotorEx.class, "motor");

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

                telemetry.addData("Encoder Position: ", motor.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}
