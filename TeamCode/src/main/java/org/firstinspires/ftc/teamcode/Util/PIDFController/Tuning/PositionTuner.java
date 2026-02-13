package org.firstinspires.ftc.teamcode.Util.PIDFController.Tuning;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Util.PIDFController.Coefficients;
import org.firstinspires.ftc.teamcode.Util.PIDFController.ControllerStates;
import org.firstinspires.ftc.teamcode.Util.PIDFController.PositionController;

@Disabled
@Configurable
@TeleOp(name = "Position PIDF Tuner", group = "Tuning")
public class PositionTuner extends OpMode {

    @IgnoreConfigurable
    private TelemetryManager telemetryM;
    private PositionController pController;

    // Conversion Parameters
    public static double ConversionUnit = 0.011560694;

    // PIDF Parameters
    public static double Kp = 0.007;
    public static double Ki = 0.0;
    public static double Kd = 0.00001;
    public static double Kf = 0.01;

    // End Velocity Error
    public static int EndPositionError = 15;

    // End State
    public static ControllerStates EndState = ControllerStates.STOP_CONTROLLER;

    // Target for FTC Dash
    public static double TargetPosition = 0.0;

    DcMotorEx motor;
    Coefficients.PositionCoefficients coefficients = new Coefficients.PositionCoefficients(
            Kp,
            Ki,
            Kd,
            Kf,
            ConversionUnit
    );

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        motor = hardwareMap.get(DcMotorEx.class, "transferLiftMotor");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        pController = new PositionController.Builder()
                .setCoefficients(coefficients)
                .setEndPositionError(EndPositionError)
                .setEndState(EndState)
                .setMotor(motor)
                .build();
    }

    @Override
    public void loop() {

        coefficients = new Coefficients.PositionCoefficients(
                Kp,
                Ki,
                Kd,
                Kf,
                ConversionUnit
        );

        pController.setTargetPosition(TargetPosition);
        pController.setCoefficients(coefficients);

        if (gamepad1.a) {
            pController.startController();
        }
        if (gamepad1.b) {
            pController.stopController();
        }

        telemetryM.addLine("Press X To Run Controller");
        telemetryM.addData("Target Position", TargetPosition);
        telemetryM.addData("Current Position", pController.getCurrentPosition());
        telemetryM.addLine("");
        telemetryM.addData("Motor Position", motor.getCurrentPosition());
        telemetryM.addData("Reference", pController.getReference());
        telemetryM.addLine("");
        telemetryM.addData("Controller Out", pController.getOut());
        telemetryM.addData("Controller State", pController.getState());
        telemetryM.addData("Thread State", pController.getThreadState());
        telemetryM.addData("Thread Loop Time", pController.getThreadLoopTime());

        telemetry = telemetryM.getWrapper();
        telemetryM.update();
        telemetry.update();
    }
}