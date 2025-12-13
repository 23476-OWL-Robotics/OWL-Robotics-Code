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
import org.firstinspires.ftc.teamcode.Util.PIDFController.PositionController;
import org.firstinspires.ftc.teamcode.Util.PIDFController.ControllerStates;

@Disabled
@Configurable
@TeleOp(name = "Controller Test", group = "Tests")
public class PositionControllerExample extends OpMode {

    DcMotorEx testMotor;
    PositionController pController;
    TelemetryManager telemetryM;

    public static double Target = 0.0;

    @Override
    public void init() {

        testMotor = hardwareMap.get(DcMotorEx.class, "motor");

        testMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pController = new PositionController.Builder()
                .setCoefficients(new Coefficients.PositionCoefficients.ExampleCoefficients())
                .setEndState(ControllerStates.HOLD_CONTROLLER)
                .setEndPositionError(5)
                .build();

        pController.setState(ControllerStates.STOP_CONTROLLER);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void loop() {
        pController.runController(testMotor.getCurrentPosition());

        if (gamepad1.a) {
            pController.setTargetPosition(Target);
            pController.setState(ControllerStates.RUN_CONTROLLER);
        }

        if (gamepad1.y) {
            pController.setState(ControllerStates.STOP_CONTROLLER);
        }

        if (gamepad1.b) {
            pController.setState(ControllerStates.HOLD_CONTROLLER);
        }

        testMotor.setPower(pController.getOut());

        telemetryM.addData("Controller Reference", pController.getReference());
        telemetryM.addData("Controller State", pController.getState());
        telemetryM.addData("Motor Position", testMotor.getCurrentPosition());
        telemetryM.addData("Motor Power", pController.getOut());

        telemetry = telemetryM.getWrapper();
        telemetry.update();
        telemetryM.update();
    }
}
