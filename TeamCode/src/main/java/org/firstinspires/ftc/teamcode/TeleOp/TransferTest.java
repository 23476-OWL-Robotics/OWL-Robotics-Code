package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Util.GamepadMappings;
import org.firstinspires.ftc.teamcode.Util.Mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.Util.Utilities;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Disabled
@Configurable
@TeleOp(name = "Transfer Test", group = "Tests")
public class TransferTest extends OpMode {

    Transfer transfer;

    GamepadMappings m;
    TelemetryManager telemetryM;

    Timer elapsedTimer;

    DcMotorEx leftLaunchMotor;
    DcMotorEx rightLaunchMotor;

    int loopTime = 50;
    long elapsedTime = 0;
    boolean controlTime = false;

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

        elapsedTime = elapsedTimer.getElapsedTime();

        if (gamepad1.dpad_left) {
            controlTime = true;
        } else if (gamepad1.dpad_right) {
            controlTime = false;
        }

        if (controlTime && elapsedTime < loopTime) {
            try {
                TimeUnit.MILLISECONDS.sleep(loopTime - elapsedTime);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        telemetry.addData("Elapsed Time", elapsedTimer.getElapsedTime());
        telemetry.addData("Limiting Time", controlTime);
        telemetry.addLine();
        transfer.Telemetry();
        telemetry.update();
        telemetryM.update(telemetry);
    }
}
