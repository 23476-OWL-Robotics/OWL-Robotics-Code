package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utilities.Auto.Arm;
import org.firstinspires.ftc.teamcode.Utilities.Auto.Intake;

@Autonomous
public class Red3Sample extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Create drive, arm, and intake for rr, arm functions, and intake functions.
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-8.5, -62, Math.toRadians(180)));
        Arm arm = new Arm(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        // Starting Positions for each action
        Pose2d action1Pose = new Pose2d(-8.5, -62, Math.toRadians(180));
        Pose2d action2Pose = new Pose2d(-8.5, -33, Math.toRadians(90));
        Pose2d action3Pose = new Pose2d(-48, -44, Math.toRadians(-90));
        Pose2d action4Pose = new Pose2d(-56, -56, Math.toRadians(-135));
        Pose2d action5Pose = new Pose2d(-58, -44, Math.toRadians(-90));
        Pose2d action6Pose = new Pose2d(-56, -56, Math.toRadians(-135));
        Pose2d action7Pose = new Pose2d(-54, -40, Math.toRadians(-45));
        Pose2d action8Pose = new Pose2d(-56, -56, Math.toRadians(-135));

        // Actions for rr
        Action action1 = drive.actionBuilder(action1Pose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-8.5, -33, Math.toRadians(90)), Math.toRadians(90))
                .build();
        Action action2 = drive.actionBuilder(action2Pose)
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(-48, -44, Math.toRadians(-90)), Math.toRadians(180))
                .build();
        Action action3 = drive.actionBuilder(action3Pose)
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(-135)), Math.toRadians(-135))
                .build();
        Action action4 = drive.actionBuilder(action4Pose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-58, -44, Math.toRadians(-90)), Math.toRadians(90))
                .build();
        Action action5 = drive.actionBuilder(action5Pose)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(-135)), Math.toRadians(-90))
                .build();
        Action action6 = drive.actionBuilder(action6Pose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-54, -40, Math.toRadians(-45)), Math.toRadians(90))
                .build();
        Action action7 = drive.actionBuilder(action7Pose)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(-135)), Math.toRadians(-90))
                .build();
        Action action8 = drive.actionBuilder(action8Pose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-24, 12, Math.toRadians(180)), Math.toRadians(0))
                .build();

        // opModeInInit and opModeIsActive
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
                            action8
                    )
            );
        }
    }
}
