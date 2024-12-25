package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utilities.Auto.Arm;
import org.firstinspires.ftc.teamcode.Utilities.Auto.Intake;

@Autonomous
public class Red extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Create drive, arm, and intake for rr, arm functions, and intake functions.
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-16.5, -62, Math.toRadians(180)));
        Arm arm = new Arm(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        // Starting Positions for each action
        Pose2d action1Pose = new Pose2d(-16.5, -62, Math.toRadians(180));
        Pose2d action2Pose = new Pose2d(-10.5, -29, Math.toRadians(90));
        Pose2d action3Pose = new Pose2d(-49, -44, Math.toRadians(-90));
        Pose2d action4Pose = new Pose2d(-58, -58, Math.toRadians(-135));
        Pose2d action5Pose = new Pose2d(-58, -44, Math.toRadians(-90));
        Pose2d action8Pose = new Pose2d(-58, -58, Math.toRadians(-135));

        Pose2d basketActionPose = new Pose2d(-55, -55, Math.toRadians(-135));

        // Actions for rr
        Action action1 = drive.actionBuilder(action1Pose)
                .waitSeconds(0.5)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-10.5, -29, Math.toRadians(90)), Math.toRadians(90))
                .build();
        Action action2 = drive.actionBuilder(action2Pose)
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(-49, -44, Math.toRadians(-90)), Math.toRadians(180))
                .build();
        Action action3 = drive.actionBuilder(action3Pose)
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(-135)), Math.toRadians(-90))
                .build();
        Action action4 = drive.actionBuilder(action4Pose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-58, -44, Math.toRadians(-90)), Math.toRadians(90))
                .build();
        Action action5 = drive.actionBuilder(action5Pose)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(-135)), Math.toRadians(-90))
                .build();
        Action action6 = drive.actionBuilder(action8Pose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-24, 12, Math.toRadians(180)), Math.toRadians(0))
                .build();

        Action basketAction = drive.actionBuilder(basketActionPose)
                .setTangent(-135)
                .splineToLinearHeading(new Pose2d(-58, -58, Math.toRadians(-135)), Math.toRadians(-135))
                .build();
        Action basketAction2 = drive.actionBuilder(basketActionPose)
                .setTangent(-135)
                .splineToLinearHeading(new Pose2d(-58, -58, Math.toRadians(-135)), Math.toRadians(-135))
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
                            new ParallelAction(
                                    arm.armUpSpecimen(),
                                    action1
                            ),
                            arm.releaseSpecimen(),
                            new ParallelAction(
                                    arm.armDown(),
                                    action2
                            ),
                            intake.intakeSample(),
                            new ParallelAction(
                                    action3,
                                    new SequentialAction(
                                            intake.intakeIn(),
                                            intake.transferSample(),
                                            arm.armUpHigh()
                                    )
                            ),
                            basketAction,
                            arm.releaseSample(),
                            new ParallelAction(
                                    arm.armDown(),
                                    new SequentialAction(
                                            action4,
                                            intake.intakeSample()
                                    )
                            ),
                            new ParallelAction(
                                    action5,
                                    new SequentialAction(
                                            intake.intakeIn(),
                                            intake.transferSample(),
                                            arm.armUpHigh()
                                    )
                            ),
                            basketAction2,
                            arm.releaseSample(),
                            new ParallelAction(
                                    action6,
                                    arm.armDown()
                            )
                    )
            );
        }
    }
}
