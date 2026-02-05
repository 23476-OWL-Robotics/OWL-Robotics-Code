package org.firstinspires.ftc.teamcode.Auto;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.TeleOp.Main.BlueTeleOp;
import org.firstinspires.ftc.teamcode.Util.ArtifactType;
import org.firstinspires.ftc.teamcode.Util.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Util.Mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.Util.Mechanisms.Lights;
import org.firstinspires.ftc.teamcode.Util.Mechanisms.Transfer;
import org.firstinspires.ftc.teamcode.Util.Paths.AutoPaths;
import org.firstinspires.ftc.teamcode.Util.Timer;
import org.firstinspires.ftc.teamcode.Util.Utilities;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@Autonomous(name = "Blue Goal", group = "Blue", preselectTeleOp = "Blue TeleOp")
public class BlueAutoGoal extends OpMode {

    private enum PathState {
        Start,
        ViewObelisk,
        ScorePreloads,
        GrabPickup1,
        ScorePickup1,
        End
    }

    PathState state;

    Follower follower;

    Lights lights;
    Intake intake;
    Transfer transfer;
    Launcher launcher;

    AutoPaths.Auto_Blue_Goal paths;
    AprilTagProcessor tagProcessor;

    final double intakeSpeed = 0.32;
    final double regularSpeed = 0.85;
    boolean lineup = false;
    boolean foundTag = false;

    Timer obeliskTimer;
    Timer launcherWarmupTimer;
    Timer endTimer;
    com.pedropathing.util.Timer loopTime;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(AutoPaths.Auto_Blue_Goal.startPose);
        follower.setMaxPower(regularSpeed);
        paths = new AutoPaths.Auto_Blue_Goal(follower);

        state = PathState.Start;

        obeliskTimer = new Timer();
        launcherWarmupTimer = new Timer();
        endTimer = new Timer();
        loopTime = new com.pedropathing.util.Timer();

        intake = new Intake(this);
        transfer = new Transfer(this);
        launcher = new Launcher(this);
        lights = new Lights(this);

        intake.init();
        intake.stopIntake();

        transfer.init();
        transfer.setPreloads(ArtifactType.PURPLE, ArtifactType.GREEN, ArtifactType.PURPLE);
        transfer.setPattern(Utilities.PatternID_22);
        transfer.setState(Transfer.TransferState.Outtake);

        launcher.init();
        launcher.setLauncherEnabled(false);
        launcher.setTargetGoal(Utilities.BlueGoalPose);

        lights.init();
        lights.Set_Purple();

        tagProcessor = launcher.getTagProcessor();
    }

    public void loop() {
        loopTime.resetTimer();
        follower.update();

        BlueTeleOp.Start_Pose = follower.getPose();

        transfer.loop();
        launcher.loop(follower.getPose());

        loopTimers();

        switch (state) {
            case Start: StartFunction(); break;
            case ViewObelisk: ObeliskFunction(); break;
            case ScorePreloads: PreloadsFunction(); break;
            case GrabPickup1: Pickup1Function(); break;
            case ScorePickup1: Score1Function(); break;
            case End: EndFunction(); break;
        }
        telemetry.addData("loopTime", loopTime.getElapsedTime());
        telemetry.update();
    }

    // When the OpMode Starts, start to move to score the Preloads, then move to the next state.
    void StartFunction() {
        follower.followPath(paths.viewObelisk, true);
        setPathState(PathState.ViewObelisk);
    }

    void ObeliskFunction() {
        if (follower.isBusy()) {
            obeliskTimer.setMillisecondTimer(2000);
            return;
        }

        ArrayList<AprilTagDetection> detections = tagProcessor.getDetections();

        if (!detections.isEmpty() && !foundTag) {

            detections.removeIf(d -> d.id == 24 || d.id == 25);

            switch (tagProcessor.getDetections().get(0).id) {
                case 21: transfer.setPattern(Utilities.PatternID_21); BlueTeleOp.Obelisk_Pattern = Utilities.PatternID_21; break;
                case 22: transfer.setPattern(Utilities.PatternID_22); BlueTeleOp.Obelisk_Pattern = Utilities.PatternID_22; break;
                case 23: transfer.setPattern(Utilities.PatternID_23); BlueTeleOp.Obelisk_Pattern = Utilities.PatternID_23; break;
            }
            foundTag = true;
        }

        if (obeliskTimer.isFinished() || foundTag) {
            launcher.setLauncherEnabled(true);

            follower.followPath(paths.scorePreloads, true);
            setPathState(PathState.ScorePreloads);
        }
    }

    // Once the follower is done, cycle through and launch the current artifacts.
    void PreloadsFunction() {
        if (follower.isBusy()) {
            launcherWarmupTimer.setMillisecondTimer(1200);
            return;
        }

        if (launcherWarmupTimer.isFinished() && transfer.CanMove() && transfer.getState() == Transfer.TransferState.Outtake) {
            transfer.EjectSelectedArtifact();
        }

        if (transfer.getState() == Transfer.TransferState.Intake) {
            lineup = false;

            launcher.setLauncherEnabled(false);

            intake.startIntake();

            follower.followPath(paths.lineupPickup1, true);
            setPathState(PathState.GrabPickup1);
        }
    }

    // Grabs the first three artifacts
    void Pickup1Function() {
        if (follower.isBusy()) {
            return;
        }

        if (!lineup) {
            follower.setMaxPower(intakeSpeed);
            follower.followPath(paths.grabPickup1, true);
            lineup = true;
            return;
        }

        launcher.setLauncherEnabled(true);

        follower.setMaxPower(regularSpeed);
        follower.followPath(paths.scorePickup1, true);
        setPathState(PathState.ScorePickup1);
    }

    void Score1Function() {
        if (follower.isBusy()) {
            launcherWarmupTimer.setMillisecondTimer(1000);
            return;
        }

        if (launcherWarmupTimer.isFinished() && transfer.CanMove() && transfer.getState() == Transfer.TransferState.Outtake) {
            intake.stopIntake();
            transfer.EjectSelectedArtifact();
        }

        if (transfer.getState() == Transfer.TransferState.Intake) {
            lineup = false;

            launcher.setLauncherEnabled(false);

            follower.followPath(paths.park, true);
            setPathState(PathState.End);
        }
    }

    void EndFunction() {
        if (follower.isBusy()) {
            endTimer.setMillisecondTimer(1000);
            return;
        }

        if (endTimer.isFinished()) {
            BlueTeleOp.Start_Pose = follower.getPose();
            terminateOpModeNow();
        }
    }

    void setPathState(PathState state) {
        this.state = state;
    }

    void loopTimers() {
        obeliskTimer.loop();
        launcherWarmupTimer.loop();
        endTimer.loop();
    }
}
