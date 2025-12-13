package org.firstinspires.ftc.teamcode.Util.PIDFController.Tuning;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Util.PIDFController.Coefficients;
import org.firstinspires.ftc.teamcode.Util.PIDFController.ControllerStates;
import org.firstinspires.ftc.teamcode.Util.PIDFController.VelocityController;

@Disabled
@Configurable
@TeleOp(name = "Velocity Controller Example", group = "Tests")
public class VelocityControllerExample extends OpMode {

    DcMotorEx testMotor;
    VelocityController vController;
    TelemetryManager telemetryM;

    public static int TargetRPM = 0;

    @Override
    public void init() {

        testMotor = hardwareMap.get(DcMotorEx.class, "testMotor");

        testMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        vController = new VelocityController.Builder()
                .setCoefficients(new Coefficients.VelocityCoefficients.ExampleCoefficients())
                .setEndState(ControllerStates.HOLD_CONTROLLER)
                .setEndVelocityError(20)
                .build();

        vController.setState(ControllerStates.STOP_CONTROLLER);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void loop() {
        vController.runController(testMotor.getVelocity());

        if (gamepad1.a) {
            vController.setTargetRPM(TargetRPM);
            vController.setState(ControllerStates.RUN_CONTROLLER);
        }

        if (gamepad1.y) {
            vController.setState(ControllerStates.STOP_CONTROLLER);
        }

        if (gamepad1.b) {
            vController.setState(ControllerStates.HOLD_CONTROLLER);
        }

        testMotor.setPower(vController.getOut());

        telemetryM.addData("Controller Reference", vController.getReference());
        telemetryM.addData("Controller State", vController.getState());
        telemetryM.addData("Motor Velocity", testMotor.getVelocity());
        telemetryM.addData("Motor RPM", vController.getCurrentRPM());
        telemetryM.addData("Motor Power", vController.getOut());

        telemetry = telemetryM.getWrapper();
        telemetry.update();
        telemetryM.update();
    }
}
