package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Util.GamepadMappings;
import org.firstinspires.ftc.teamcode.Util.Mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.Util.Utilities;

@Disabled
@Configurable
@TeleOp(name = "New Transfer", group = "Tests")
public class NewTransfer extends OpMode {

    Transfer transfer;

    GamepadMappings m;
    TelemetryManager telemetryM;

    Timer elapsedTimer;

    @Override
    public void init() {
        m = new GamepadMappings(this);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        transfer = new Transfer(this);

        transfer.init();
        transfer.setState(Transfer.TransferState.Intake);
        transfer.setPattern(Utilities.PatternID_22);

        elapsedTimer = new Timer();
    }

    @Override
    public void loop() {
        elapsedTimer.resetTimer();
        transfer.loop();

        if (transfer.getState() == Transfer.TransferState.Outtake && m.gamepad1.a) {
            transfer.EjectSelectedArtifact();
        }

        transfer.Telemetry();
        telemetry.addData("Elapsed Timer", elapsedTimer.getElapsedTime());
        telemetry.update();
        telemetryM.update(telemetry);
    }
}
