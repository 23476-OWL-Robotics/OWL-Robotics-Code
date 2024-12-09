package org.firstinspires.ftc.teamcode.Utilities.PIDF_Controller;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp
public class PIDF_Tuner extends OpMode {

    // Conversion Parameters
    double rotPerTick = 0.0;
    double inPerTick = 0.006371562;
    double degPerTick = 0.0;

    // PIDF Parameters
    public static double Kp = 0.0;
    public static double Ki = 0.0;
    public static double Kd = 0.0;
    public static double Kf = 0.0;

    // Target for FTC Dashboard
    public static double target = 0.0;

    DcMotorEx motor;

    // Tuner doubles
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
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {

        // Get encoder reference
        reference = target / inPerTick;
        // Get encoder position
        encoderPosition = motor.getCurrentPosition();

        // find the error
        error = reference - encoderPosition;
        // rate of change of the error
        derivative = (error - lastError) / timer.seconds();

        // sum of all error over time
        integralSum = integralSum + (error * timer.seconds());

        // feedForward for gravity counteraction
        feedForward = Math.cos(Math.toRadians(reference));

        // Motor out
        out = (Kp * error) + (Ki * integralSum) + (Kd * derivative) + (Kf * feedForward);

        // Keep the motor from going past 60% power
        motor.setPower(Math.min(out, 0.8));

        // Set lastError
        lastError = error;

        // reset the timer for next time
        timer.reset();

        // Telemetry for FTC Dash
        telemetry.addData("Reference ", reference);
        telemetry.addData("Position ", encoderPosition);
        telemetry.addLine();
        telemetry.addData("Kp ", Kp * error);
        telemetry.addData("Ki ", Ki * integralSum);
        telemetry.addData("Kd ", Kd * derivative);
        telemetry.addData("Kf ", Kf * feedForward);
        telemetry.addData("Out: ", out);
        telemetry.update();
    }
}
