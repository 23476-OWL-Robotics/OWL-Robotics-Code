package org.firstinspires.ftc.teamcode.Other.DcMotorEx;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Config
@TeleOp
public class DcMotorEx_Test extends LinearOpMode {

    public static double p = 2.0;
    public static double i = 0.0;
    public static double d = 0.0;
    public static double f = 0.0;

    public static int Position = 1000;

    DcMotorEx motor1;

    PIDFCoefficients pidfCoefficients = new PIDFCoefficients(p, i, d, f);

    @Override
    public void runOpMode() {

        motor1 = hardwareMap.get(DcMotorEx.class, "motor");
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while (opModeIsActive()) {
            motor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

            if (motor1.getCurrentPosition() > Position + 5 || motor1.getCurrentPosition() < Position - 5) {
                motor1.setPower(1);
                motor1.setTargetPosition(Position);
                motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else {
                motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            telemetry.addData("Position: ", motor1.getCurrentPosition());
            telemetry.addData("Coefficients: ", motor1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER));
            telemetry.update();
        }
    }
}
