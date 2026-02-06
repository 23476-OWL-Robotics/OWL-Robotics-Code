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

//@Disabled
@Configurable
@TeleOp(name = "Position PIDF Tuner", group = "Tuning")
public class PositionTuner extends OpMode {

    @IgnoreConfigurable
    private TelemetryManager telemetryM;
    private PositionController pController;

    // Conversion Parameters
    public static double ConversionUnit = 0.011560694;

    // PIDF Parameters
    public static double Kp = 0.0;
    public static double Ki = 0.0;
    public static double Kd = 0.0;
    public static double Kf = 0.0;

    // End Velocity Error
    public static int EndPositionError = 15;

    // End State
    public static ControllerStates EndState = ControllerStates.RUN_CONTROLLER;
    public static ControllerStates CurrentState = ControllerStates.RUN_CONTROLLER;

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

        pController = new PositionController.Builder()
                .setCoefficients(coefficients)
                .setEndPositionError(EndPositionError)
                .setEndState(EndState)
                .build();

        motor = hardwareMap.get(DcMotorEx.class, "transferLiftMotor");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
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

        pController.setCoefficients(coefficients);
        pController.setTargetPosition(TargetPosition);

        pController.setState(CurrentState);

        pController.runController(motor.getCurrentPosition());

        motor.setPower(pController.getOut());

        telemetryM.addData("Target Position", pController.getTargetPosition());
        telemetryM.addData("Current Position", pController.getCurrentPosition());
        telemetryM.addLine("");
        telemetryM.addData("Motor Position", pController.getEncoderPosition());
        telemetryM.addData("Reference", pController.getReference());
        telemetryM.addLine("");
        telemetryM.addData("Controller Out", pController.getOut());
        telemetryM.addData("Controller State", pController.getState());

        telemetry = telemetryM.getWrapper();
        telemetryM.update();
        telemetry.update();
    }
}