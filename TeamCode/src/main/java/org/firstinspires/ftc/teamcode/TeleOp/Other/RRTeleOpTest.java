package org.firstinspires.ftc.teamcode.TeleOp.Other;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

import java.util.ArrayList;
import java.util.List;

@Disabled
@TeleOp
public class RRTeleOpTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));
        FtcDashboard dash = FtcDashboard.getInstance();

        List<Action> runningActions = new ArrayList<>();

        Action test1 = drive.actionBuilder(drive.pose)
                .lineToX(24)
                .build();
        Action test2 = drive.actionBuilder(drive.pose)
                .lineToX(0)
                .build();

        if (opModeIsActive()) {
            while (opModeIsActive()){

                TelemetryPacket packet = new TelemetryPacket();

                // updated based on gamepads
                if (gamepad1.a) {
                    runningActions.add(new SequentialAction(
                            test1
                    ));
                }
                if (gamepad1.b) {
                    runningActions.add(new SequentialAction(
                            test2
                    ));
                }

                // update running actions
                List<Action> newActions = new ArrayList<>();
                for (Action action : runningActions) {
                    action.preview(packet.fieldOverlay());
                    if (action.run(packet)) {
                        newActions.add(action);
                    }
                }
                runningActions = newActions;

                dash.sendTelemetryPacket(packet);
                drive.updatePoseEstimate();
            }
        }
    }
}
