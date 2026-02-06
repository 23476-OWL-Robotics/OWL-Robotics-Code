package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Util.GamepadMappings;
import org.firstinspires.ftc.teamcode.Util.Mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.Util.Utilities;

@Configurable
@TeleOp(name = "Transfer Test", group = "Tests")
public class TransferTest extends OpMode {

    Transfer transfer;

    GamepadMappings m;
    TelemetryManager telemetryM;

    Timer elapsedTimer;

    DcMotorEx leftLaunchMotor;
    DcMotorEx rightLaunchMotor;

    @Override
    public void init() {
        m = new GamepadMappings(this);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        transfer = new Transfer(this);

        transfer.init();
        transfer.setState(Transfer.TransferState.Intake);
        transfer.setPattern(Utilities.PatternID_22);

        leftLaunchMotor = hardwareMap.get(DcMotorEx.class, "launcherLeftMotor");
        rightLaunchMotor = hardwareMap.get(DcMotorEx.class, "launcherRightMotor");

        rightLaunchMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLaunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightLaunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        elapsedTimer = new Timer();
    }

    @Override
    public void init_loop() {
        if (transfer.ZeroLiftMotor()) return;
        transfer.Telemetry();
        telemetry.update();
    }

    @Override
    public void loop() {
        elapsedTimer.resetTimer();
        transfer.loop();

        if (transfer.getState() == Transfer.TransferState.Outtake && m.gamepad1.a) {
            transfer.EjectSelectedArtifact();
        }

        if (m.gamepad1.b) {
            transfer.ZeroLiftMotor_Running();
        }

        if (transfer.getState() == Transfer.TransferState.Outtake) {
            leftLaunchMotor.setPower(0.4);
            rightLaunchMotor.setPower(0.4);
        } else {
            leftLaunchMotor.setPower(0);
            rightLaunchMotor.setPower(0);
        }

        transfer.Telemetry();
        telemetry.addData("Elapsed Timer", elapsedTimer.getElapsedTime());
        telemetry.update();
        telemetryM.update(telemetry);
    }
}
