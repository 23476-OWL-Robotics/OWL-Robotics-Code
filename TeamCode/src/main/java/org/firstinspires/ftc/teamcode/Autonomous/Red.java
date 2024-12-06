package org.firstinspires.ftc.teamcode.Autonomous;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utilities.Auto.Arm;
import org.firstinspires.ftc.teamcode.Utilities.Auto.Intake;
import org.firstinspires.ftc.teamcode.Utilities.Auto.PoseUpdate;
import org.firstinspires.ftc.teamcode.Utilities.Auto.RedTrajectories;

@Autonomous
public class Red extends LinearOpMode {

    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(8.5 ,-60, Math.toRadians(0)));
        Arm arm = new Arm(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        Pose2d action1Pose = new Pose2d(8.5, -60, Math.toRadians(0));
        Pose2d action2Pose = new Pose2d(8.5, -30, Math.toRadians(90));
        Pose2d action3Pose = new Pose2d(38, -38, Math.toRadians(-135));
        Pose2d action4Pose = new Pose2d(-55, -55, Math.toRadians(-135));
        Pose2d action5Pose = new Pose2d(-48, -48, Math.toRadians(-90));
        Pose2d action6Pose = new Pose2d(-55, -55, Math.toRadians(-135));
        Pose2d action7Pose = new Pose2d(-58, -47, Math.toRadians(-90));
        Pose2d action8Pose = new Pose2d(-55, -55, Math.toRadians(-135));
        Pose2d action9Pose = new Pose2d(-54, -40, Math.toRadians(-45));
        Pose2d action10Pose = new Pose2d(-55, -55, Math.toRadians(-135));

        Action action1 = drive.actionBuilder(action1Pose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(8.5, -30, Math.toRadians(90)), Math.toRadians(90))
                .build();
        Action action2 = drive.actionBuilder(action2Pose)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(38, -38, Math.toRadians(-135)), Math.toRadians(0))
                .build();
        Action action3 = drive.actionBuilder(action3Pose)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(-135)), Math.toRadians(180))
                .build();
        Action action4 = drive.actionBuilder(action4Pose)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-48, -48, Math.toRadians(-90)), Math.toRadians(45))
                .build();
        Action action5 = drive.actionBuilder(action5Pose)
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(-135)), Math.toRadians(-135))
                .build();
        Action action6 = drive.actionBuilder(action6Pose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-58, -47, Math.toRadians(-90)), Math.toRadians(90))
                .build();
        Action action7 = drive.actionBuilder(action7Pose)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(-135)), Math.toRadians(-90))
                .build();
        Action action8 = drive.actionBuilder(action8Pose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-54, -40, Math.toRadians(-45)), Math.toRadians(90))
                .build();
        Action action9 = drive.actionBuilder(action9Pose)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(-135)), Math.toRadians(-90))
                .build();
        Action action10 = drive.actionBuilder(action10Pose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-24, 12, Math.toRadians(180)), Math.toRadians(0))
                .build();

        while (opModeInInit()) {
            arm.init();
            intake.init();
        }
        waitForStart();
        if (opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(
                            action1,
                            action2,
                            action3,
                            action4,
                            action5,
                            action6,
                            action7,
                            action8,
                            action9,
                            action10
                    )
            );
        }
    }
}
