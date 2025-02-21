package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utilities.Auto.Arm;
import org.firstinspires.ftc.teamcode.Utilities.Auto.Intake;

@Autonomous(name = "Red", preselectTeleOp = "Red TeleOp")
public class RedSample extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Create drive, arm, and intake for rr, arm functions, and intake functions.
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-16.5, -62, Math.toRadians(180)));
        Arm arm = new Arm(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        // Starting Positions for each action
        Pose2d action0Pose = new Pose2d(-16.5, -62, Math.toRadians(180));
        Pose2d action1Pose = new Pose2d(-10.5, -30, Math.toRadians(90));
        Pose2d action2Pose = new Pose2d(-50, -44, Math.toRadians(-90));
        Pose2d action3Pose = new Pose2d(-58, -58, Math.toRadians(-135));
        Pose2d action4Pose = new Pose2d(-60, -44, Math.toRadians(-90));
        Pose2d action5Pose = new Pose2d(-58, -58, Math.toRadians(-135));
        Pose2d action6Pose = new Pose2d(-61, -39, Math.toRadians(-60));
        Pose2d action7Pose = new Pose2d(-56, -56, Math.toRadians(-135));

        Pose2d basketActionPose = new Pose2d(-55, -55, Math.toRadians(-135));

        // Actions for rr
        Action action0 = drive.actionBuilder(action0Pose)
                .waitSeconds(0.5)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-10.5, -40, Math.toRadians(90)), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-10.5, -30, Math.toRadians(90)), Math.toRadians(90))
                .build();
        Action action1 = drive.actionBuilder(action1Pose)
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-40, -44, Math.toRadians(-90)), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-50, -44), Math.toRadians(180))
                .build();
        Action action2 = drive.actionBuilder(action2Pose)
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(-135)), Math.toRadians(-90))
                .build();
        Action action3 = drive.actionBuilder(action3Pose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-60, -44, Math.toRadians(-90)), Math.toRadians(90))
                .build();
        Action action4 = drive.actionBuilder(action4Pose)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(-135)), Math.toRadians(-90))
                .build();
        Action action5 = drive.actionBuilder(action5Pose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-61, -39, Math.toRadians(-60)), Math.toRadians(90))
                .build();
        Action action6 = drive.actionBuilder(action6Pose)
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-55, -60, Math.toRadians(-135)), Math.toRadians(-90))
                .build();
        Action action7 = drive.actionBuilder(action7Pose)
                .setTangent(Math.toRadians(45))
                .splineToSplineHeading(new Pose2d(-36, -12, Math.toRadians(180)), Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-24, -12, Math.toRadians(180)), Math.toRadians(0))
                .build();

        Action basketAction1 = drive.actionBuilder(basketActionPose)
                .setTangent(-135)
                .splineToLinearHeading(new Pose2d(-58, -58, Math.toRadians(-135)), Math.toRadians(-135))
                .build();
        Action basketAction2 = drive.actionBuilder(basketActionPose)
                .setTangent(-135)
                .splineToLinearHeading(new Pose2d(-58, -58, Math.toRadians(-135)), Math.toRadians(-135))
                .build();
        Action basketAction3 = drive.actionBuilder(basketActionPose)
                .setTangent(-135)
                .splineToLinearHeading(new Pose2d(-56, -56, Math.toRadians(-135)), Math.toRadians(-135))
                .build();


        // opModeInInit and opModeIsActive
        while (opModeInInit()) {
            arm.init();
            intake.init();
        }
        waitForStart();
        if (opModeIsActive()) {

            Actions.runBlocking(
                    new ParallelAction(
                            arm.runController(),
                            new SequentialAction(
                                    new ParallelAction(//drive near bar, bring up arm
                                            action0,
                                            arm.armUpSpecimen(),
                                            arm.pivotArm()
                                    ),
                                    arm.waitTme(),
                                    arm.release(),
                                    new ParallelAction(//bring arm down, drive to intake first sample
                                            action1,
                                            arm.armDown(),
                                            arm.pivotArm()

                                    ),
                                    intake.intakeSample(),//intake the sample
                                    new ParallelAction(//drive to basket
                                            action2,
                                            new SequentialAction(
                                                    intake.intakeIn(),
                                                    intake.transferSample()
                                            )
                                    ),
                                    arm.armUpHigh(),
                                    new ParallelAction(
                                            basketAction1,
                                            arm.pivotArm()
                                    ),
                                    arm.release(),
                                    new ParallelAction(
                                            new SequentialAction(
                                                    arm.pivotArm(),
                                                    arm.armDown()

                                            ),
                                            new SequentialAction(
                                                    action3,
                                                    intake.intakeSample()
                                            )
                                    ),
                                    new ParallelAction(
                                            action4,
                                            new SequentialAction(
                                                    intake.intakeIn(),
                                                    intake.transferSample()
                                            )
                                    ),
                                    arm.armUpHigh(),
                                    new ParallelAction(
                                            arm.pivotArm(),
                                            basketAction2
                                    ),
                                    arm.release(),
                                    new ParallelAction(
                                            new SequentialAction(
                                                    arm.pivotArm(),
                                                    arm.armDown()

                                            ),
                                            new SequentialAction(
                                                    action5,
                                                    intake.intakeSampleSlow()
                                            )
                                    ),
                                    new ParallelAction(
                                            action6,
                                            new SequentialAction(
                                                    intake.intakeIn(),
                                                    intake.transferSample()
                                            )
                                    ),
                                    arm.armUpHigh(),
                                    new ParallelAction(
                                            arm.pivotArm(),
                                            basketAction3
                                    ),
                                    arm.release(),
                                    new ParallelAction(
                                            action7,
                                            new SequentialAction(
                                                    arm.pivotArm(),
                                                    arm.armDown(),
                                                    arm.toEndPositions(),
                                                    intake.toEndPositions()
                                            )
                                    ),
                                    arm.stopController()
                            )
                    )
            );
        }
    }
}
