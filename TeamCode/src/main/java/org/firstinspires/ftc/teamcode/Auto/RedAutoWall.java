package org.firstinspires.ftc.teamcode.Auto;

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
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "Red Wall", group = "Red", preselectTeleOp = "Red TeleOp")
public class RedAutoWall extends OpMode {

    private enum PathState {
        Start,
        Score,
        GrabPickup1,
        End
    }

    PathState state;

    Follower follower;

    Lights lights;
    Intake intake;
    Transfer transfer;
    Launcher launcher;

    AutoPaths.Auto_Red_Wall paths;
    AprilTagProcessor tagProcessor;

    final double intakeSpeed = 0.32;
    final double regularSpeed = 1.0;
    boolean lineup = false;

    Timer obeliskTimer;
    Timer launcherWarmupTimer;
    Timer endTimer;
    com.pedropathing.util.Timer loopTime;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(AutoPaths.Auto_Red_Wall.startPose);
        follower.setMaxPower(regularSpeed);
        paths = new AutoPaths.Auto_Red_Wall(follower);

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
        launcher.setLauncherEnabled(true);
        launcher.setTargetGoal(Utilities.RedGoalPose);

        lights.init();
        lights.Set_Purple();

        tagProcessor = launcher.getTagProcessor();
    }

    @Override
    public void init_loop() {
        if (transfer.ZeroLiftMotor()) return;
        transfer.Telemetry();
        telemetry.update();
    }

    public void loop() {
        loopTime.resetTimer();
        follower.update();

        RedTeleOp.Start_Pose = follower.getPose();

        intake.loop();
        transfer.loop();
        launcher.loop(follower.getPose());

        loopTimers();

        switch (state) {
            case Start: StartFunction(); break;
            case Score: ScoreFunction(); break;
            case GrabPickup1: Pickup1Function(); break;
            case End: EndFunction(); break;
        }

        telemetry.addData("loopTime", loopTime.getElapsedTime());
        telemetry.update();
    }

    // When the OpMode Starts, start to move to score the Preloads, then move to the next state.
    void StartFunction() {
        launcher.setLauncherEnabled(true);
        launcherWarmupTimer.setMillisecondTimer(3000);
        setPathState(PathState.Score);
    }

    // Once the follower is done, cycle through and launch the current artifacts.
    void ScoreFunction() {
        if (follower.isBusy()) {
            launcherWarmupTimer.setMillisecondTimer(3000);
            return;
        }

        if (launcherWarmupTimer.isFinished() && transfer.CanMove() && transfer.getState() == Transfer.TransferState.Outtake) {
            transfer.EjectSelectedArtifact();
        }

        if (transfer.getState() == Transfer.TransferState.Intake) {
            launcher.setLauncherEnabled(false);

            intake.startIntake();

            if (follower.getCurrentPathChain() != paths.scorePickup1) {
                follower.followPath(paths.lineupPickup1, true);
                lineup = false;
                setPathState(PathState.GrabPickup1);
            }
            if (follower.getCurrentPathChain() == paths.scorePickup1) {
                follower.followPath(paths.park, true);
                setPathState(PathState.End);
            }
        }
    }

    // Grabs the first three artifacts
    void Pickup1Function() {
        if (follower.isBusy()) {
            return;
        }

        if (!lineup) {
            follower.setMaxPower(intakeSpeed);
            follower.followPath(paths.grabArtifact1Pickup1, false);
            lineup = true;
            return;
        }

        if (follower.getCurrentPathChain() == paths.grabArtifact1Pickup1) {
            follower.followPath(paths.grabArtifact2Pickup1, false);
            return;
        }
        if (follower.getCurrentPathChain() == paths.grabArtifact2Pickup1) {
            follower.followPath(paths.grabArtifact3Pickup1, false);
            return;
        }

        launcher.setLauncherEnabled(true);

        follower.setMaxPower(regularSpeed);
        follower.followPath(paths.scorePickup1, true);
        setPathState(PathState.Score);
    }

    void EndFunction() {
        endTimer.loop();
        if (follower.isBusy()) {
            endTimer.setMillisecondTimer(800);
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
    }
}
