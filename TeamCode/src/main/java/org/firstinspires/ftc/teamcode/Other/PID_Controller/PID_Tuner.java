package org.firstinspires.ftc.teamcode.Other.PID_Controller;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp
public class PID_Tuner extends OpMode {

    double rotPerTick = 0.000695;
    double inPerTick = 0.0;

    public static double Kp = 0.00059;
    public static double Ki = 0.0;
    public static double Kd = 0.0;

    public static double target = 0.0;

    DcMotorEx motor;

    double reference;
    double integralSum = 0.0;
    double feedForward;
    double lastError = 0.0;
    int encoderPosition;
    double error;
    double derivative;
    double out;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {

        reference = target / rotPerTick;
        encoderPosition = motor.getCurrentPosition();

        // find the error
        error = reference - encoderPosition;
        // rate of change of the error
        derivative = (error - lastError) / timer.seconds();

        // sum of all error over time
        integralSum = integralSum + (error * timer.seconds());

        out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        motor.setPower(Math.min(out, 0.6));

        lastError = error;

        // reset the timer for next time
        timer.reset();

        telemetry.addData("Reference ", reference);
        telemetry.addData("Position ", encoderPosition);
        telemetry.addLine();
        telemetry.addData("Kp ", Kp * error);
        telemetry.addData("Ki ", Ki * integralSum);
        telemetry.addData("Kd ", Kd * derivative);
        telemetry.update();
    }
}
