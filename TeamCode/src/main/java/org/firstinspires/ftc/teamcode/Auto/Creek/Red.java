package org.firstinspires.ftc.teamcode.Auto.Creek;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.TeleOp.Main.RedTeleOp;
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

@Autonomous(name = "Red Goal Creek", group = "Red", preselectTeleOp = "Red TeleOp")
public class Red extends OpMode {

    private enum PathState {
        Start,
        ViewObelisk,
        ScorePreloads,
        End
    }

    PathState state;

    Follower follower;

    Lights lights;
    Intake intake;
    Transfer transfer;
    Launcher launcher;

    AutoPaths.Auto_Red_Goal paths;
    AprilTagProcessor tagProcessor;

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
        follower.setStartingPose(AutoPaths.Auto_Red_Goal.startPose);
        follower.setMaxPower(regularSpeed);
        paths = new AutoPaths.Auto_Red_Goal(follower);

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
        launcher.setTargetGoal(Utilities.RedGoalPose);

        lights.init();
        lights.Set_Purple();

        tagProcessor = launcher.getTagProcessor();
    }

    public void loop() {
        loopTime.resetTimer();
        follower.update();

        RedTeleOp.Start_Pose = follower.getPose();

        transfer.loop();
        launcher.loop(follower.getPose());

        loopTimers();

        switch (state) {
            case Start: StartFunction(); break;
            case ViewObelisk: ObeliskFunction(); break;
            case ScorePreloads: PreloadsFunction(); break;
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
            obeliskTimer.setMillisecondTimer(1000);
            return;
        }

        ArrayList<AprilTagDetection> detections = tagProcessor.getDetections();

        if (!detections.isEmpty() && !foundTag) {

            detections.removeIf(d -> d.id == 24 || d.id == 25);

            switch (tagProcessor.getDetections().get(0).id) {
                case 21: transfer.setPattern(Utilities.PatternID_21); RedTeleOp.Obelisk_Pattern = Utilities.PatternID_21; break;
                case 22: transfer.setPattern(Utilities.PatternID_22); RedTeleOp.Obelisk_Pattern = Utilities.PatternID_22; break;
                case 23: transfer.setPattern(Utilities.PatternID_23); RedTeleOp.Obelisk_Pattern = Utilities.PatternID_23; break;
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

            follower.followPath(paths.creekPark, true);
            setPathState(PathState.End);
        }
    }

    void EndFunction() {
        if (follower.isBusy()) {
            endTimer.setMillisecondTimer(1000);
            return;
        }

        if (endTimer.isFinished()) {
            RedTeleOp.Start_Pose = follower.getPose();
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
