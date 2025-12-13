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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Util.PIDFController.Coefficients;
import org.firstinspires.ftc.teamcode.Util.PIDFController.ControllerStates;
import org.firstinspires.ftc.teamcode.Util.PIDFController.VelocityController;

@Disabled
@Configurable
@TeleOp(name = "Velocity PID Tuner", group = "Tuning")
public class VelocityTuner extends OpMode {

    @IgnoreConfigurable
    private TelemetryManager telemetryM;
    private VelocityController vController;

    // Ticks Per Rev
    public static int TicksPerRev = 0;

    // PIDF Parameters
    public static double Kp = 0.0;
    public static double Ki = 0.0;
    public static double Kd = 0.0;

    // End Velocity Error
    public static int EndVelocityError = 15;

    // End State and Current State
    public static ControllerStates EndState = ControllerStates.HOLD_CONTROLLER;
    public static ControllerStates CurrentState = ControllerStates.RUN_CONTROLLER;

    // Target for FTC Dash
    public static int TargetRPM = 0;

    // Motor
    DcMotorEx motor;

    // Velocity Coefficients
    Coefficients.VelocityCoefficients coefficients = new Coefficients.VelocityCoefficients(
            Kp,
            Ki,
            Kd,
            TicksPerRev
    );

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        vController = new VelocityController.Builder()
                .setCoefficients(coefficients)
                .setEndVelocityError(EndVelocityError)
                .setEndState(EndState)
                .build();

        vController.setState(CurrentState);

        motor = hardwareMap.get(DcMotorEx.class, "launcherMotor");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        coefficients = new Coefficients.VelocityCoefficients(
                Kp,
                Ki,
                Kd,
                TicksPerRev
        );

        vController.setCoefficients(coefficients);
        vController.setTargetRPM(TargetRPM);

        vController.setState(CurrentState);

        vController.runController(motor.getVelocity());

        motor.setPower(vController.getOut());

        telemetryM.addData("Target Velocity", vController.getTargetVelocity());
        telemetryM.addData("Current Velocity", motor.getVelocity());
        telemetryM.addLine("");
        telemetryM.addData("Target RPM", vController.getTargetRPM());
        telemetryM.addData("Current RPM", vController.getCurrentRPM());
        telemetryM.addLine("");
        telemetryM.addData("Controller Out", vController.getOut());
        telemetryM.addData("Controller State", vController.getState());

        telemetry = telemetryM.getWrapper();
        telemetryM.update();
        telemetry.update();
    }
}
