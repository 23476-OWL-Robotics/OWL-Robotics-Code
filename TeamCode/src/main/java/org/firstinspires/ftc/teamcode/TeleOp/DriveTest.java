package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Util.GamepadMappings;
import org.firstinspires.ftc.teamcode.Util.Mechanisms.DriveTrain;
import org.firstinspires.ftc.teamcode.Util.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Util.Mechanisms.Launcher;
import org.firstinspires.ftc.teamcode.Util.Utilities;

@Disabled
@Configurable
@TeleOp(name = "DriveTest", group = "Tests")
public class DriveTest extends OpMode {

    Follower follower;

    DriveTrain driveTrain;
    Intake intake;
    Launcher launcher;

    GamepadMappings mappings;

    Timer loopTime;

    @Override
    public void init() {
        mappings = new GamepadMappings(this);
        driveTrain = new DriveTrain(this);
        launcher = new Launcher(this);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(110, 135.5, Math.toRadians(270)));
        follower.isLocalizationNAN();

        driveTrain.init(0);

        intake = new Intake(this);

        intake.init();
        intake.stopIntake();

        launcher.init();
        launcher.setTargetGoal(Utilities.RedGoalPose);

        loopTime = new Timer();
    }

    @Override
    public void loop() {
        loopTime.resetTimer();
        follower.update();

        driveTrain.FieldCentric(mappings);
        launcher.loop(follower.getPose());

        launcher.Telemetry();
        telemetry.addLine();
        intake.Telemetry();
        telemetry.addLine();

        telemetry.addData("Heading", driveTrain.getHeading());
        telemetry.addLine("----Localization----");
        telemetry.addData("Pinpoint Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Robot X Pose", follower.getPose().getX());
        telemetry.addData("Robot Y Pose", follower.getPose().getY());
        telemetry.addLine();
        telemetry.addData("Loop Time", loopTime.getElapsedTime());
        telemetry.update();
    }
}
