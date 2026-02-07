package org.firstinspires.ftc.teamcode.TeleOp.Main;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Util.ArtifactPattern;
import org.firstinspires.ftc.teamcode.Util.ArtifactType;
import org.firstinspires.ftc.teamcode.Util.GamepadMappings;
import org.firstinspires.ftc.teamcode.Util.Mechanisms.DriveTrain;
import org.firstinspires.ftc.teamcode.Util.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Util.Mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.Util.Mechanisms.Lights;
import org.firstinspires.ftc.teamcode.Util.Mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.Util.Paths.AutoPaths;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.Util.Utilities;

import java.util.List;

@TeleOp(name = "Red TeleOp", group = "Main")
public class RedTeleOp extends OpMode {

    public static Pose Start_Pose = AutoPaths.Auto_Red_Goal.startPose;
    public static ArtifactPattern Obelisk_Pattern = new ArtifactPattern(ArtifactType.PURPLE, ArtifactType.GREEN, ArtifactType.PURPLE);

    GamepadMappings m;

    Lights lights;
    DriveTrain driveTrain;
    Intake intake;
    Transfer transfer;
    Launcher launcher;

    Follower follower;

    boolean isRunning = false;
    boolean launchArtifacts = false;
    boolean canRumble = true;

    Timer stateChangeTimer;
    com.pedropathing.util.Timer loopLime;
    List<LynxModule> allHubs;

    @Override
    public void init() {
        m = new GamepadMappings(this);

        lights = new Lights(this);
        driveTrain = new DriveTrain(this);
        intake = new Intake(this);
        transfer = new Transfer(this);
        launcher = new Launcher(this);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Start_Pose);

        lights.init();
        lights.Set_Red();

        driveTrain.init(Math.toDegrees(Start_Pose.getHeading()) + Utilities.RedTeleHeadingOffset);

        intake.init();
        intake.stopIntake();

        launcher.init();
        launcher.setTargetGoal(Utilities.RedGoalPose);

        stateChangeTimer = new Timer();
        loopLime = new com.pedropathing.util.Timer();

        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    @Override
    public void loop() {
        loopLime.resetTimer();

        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }

        if (!isRunning) {
            isRunning = m.Start_OpMode();

            if (isRunning) {
                transfer.init();
                transfer.setState(Transfer.TransferState.Intake);
                transfer.setPattern(Obelisk_Pattern);

                transfer.ZeroLiftMotor_Running();
            }
            lights.Set_Red();
            return;
        }

        follower.update();
        Start_Pose = follower.getPose();

        driveTrain.FieldCentric(m);
        intake.loop();
        transfer.loop();
        launcher.loop(follower.getPose());

        stateChangeTimer.loop();

        switch (transfer.getState()) {
            case Intake: {
                intake.startIntake();
                launcher.setLauncherEnabled(false);

                if (canRumble) {
                    m.Rumble_Both(300);
                    lights.Set_Blue();
                    canRumble = false;
                }
            } break;
            case Outtake:  {
                intake.stopIntake();
                launcher.setLauncherEnabled(true);

                if (!canRumble) {
                    m.Rumble_Both(300);
                    lights.Set_Green();
                    canRumble = true;
                }
            }
        }

        if (m.Transfer_Eject() && transfer.getState() == Transfer.TransferState.Outtake) {
            //launchArtifacts = true;
            transfer.EjectSelectedArtifact();
        } else if (m.Transfer_Eject() && transfer.getState() == Transfer.TransferState.Intake) {
            m.Rumble_Gamepad_2(800);
        }
        LaunchAllArtifacts();

        if (m.Transfer_State_Change() && stateChangeTimer.isFinished()) {
            Transfer.TransferState oldState = transfer.getState();
            transfer.switchState();

            if (transfer.getState() == oldState) {
                m.Rumble_Gamepad_2(800);
            }
            stateChangeTimer.setMillisecondTimer(500);
        }

        if (m.Intake_Reverse_Sinner() > 0.2) {
            intake.reverseIntake();
        }

        if (m.Launcher_Reverse_Motor() > 0.2) {
            launcher.reverseMotor();
        } else {
            launcher.stopReverse();
        }

        if (m.Terminate_OpMode()) {
            intake.stopIntake();
            terminateOpModeNow();
        }

        telemetry.addData("Loop Time", loopLime.getElapsedTime());
        transfer.Telemetry();
        telemetry.update();
    }

    void LaunchAllArtifacts() {
        if (!launchArtifacts) {
            return;
        }

        // If the transfer still have artifacts, and all timers are finished, launch that artifact.
        if (transfer.getState() == Transfer.TransferState.Outtake) {
            transfer.EjectSelectedArtifact();
            return;
        }

        if (transfer.getState() == Transfer.TransferState.Intake) {
            launchArtifacts = false;
            transfer.ZeroLiftMotor_Running();
        }
    }
}
