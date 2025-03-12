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
public class SpecimenTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Create drive, arm, and intake for rr, arm functions, and intake functions.
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-16.5, 62, Math.toRadians(0)));
        Arm arm = new Arm(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        Pose2d action0Pose = new Pose2d(-16.5, 62, Math.toRadians(0));
        Pose2d action1Pose = new Pose2d(-36, 56, Math.toRadians(-45));
        Pose2d action2Pose = new Pose2d(-5, 34, Math.toRadians(-90));
        Pose2d action3Pose = new Pose2d(-36, 56, Math.toRadians(-45));
        Pose2d action4Pose = new Pose2d(-5, 34, Math.toRadians(-90));
        Pose2d action5Pose = new Pose2d(-36, 56, Math.toRadians(-45));
        Pose2d action6Pose = new Pose2d(-5, 34, Math.toRadians(-90));

        Pose2d sweepAction0Pose = new Pose2d(-5, 34, Math.toRadians(-90));
        Pose2d sweepAction1Pose = new Pose2d(-34, 38, Math.toRadians(45));
        Pose2d sweepAction2Pose = new Pose2d(-34, 48, Math.toRadians(-45));
        Pose2d sweepAction3Pose = new Pose2d(-44, 38, Math.toRadians(45));
        Pose2d sweepAction4Pose = new Pose2d(-44, 48, Math.toRadians(-45));
        Pose2d sweepAction5Pose = new Pose2d(-54, 38, Math.toRadians(45));
        Pose2d sweepAction6Pose = new Pose2d(-54, 48, Math.toRadians(-45));

        Action action0 = drive.actionBuilder(action0Pose)
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-5, 40, Math.toRadians(-90)), Math.toRadians(-90))
                .waitSeconds(0.1)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-5, 34, Math.toRadians(-90)), Math.toRadians(-90))
                .waitSeconds(0.5)
                .build();

        Action sweepAction0 = drive.actionBuilder(sweepAction0Pose)
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-34, 38, Math.toRadians(45)), Math.toRadians(180))
                .waitSeconds(0.5)
                .build();
        Action sweepAction1 = drive.actionBuilder(sweepAction1Pose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-34, 48, Math.toRadians(-45)), Math.toRadians(90))
                .waitSeconds(0.5)
                .build();
        Action sweepAction2 = drive.actionBuilder(sweepAction2Pose)
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(-44, 38, Math.toRadians(45)), Math.toRadians(-135))
                .waitSeconds(0.5)
                .build();
        Action sweepAction3 = drive.actionBuilder(sweepAction3Pose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-44, 48, Math.toRadians(-45)), Math.toRadians(90))
                .waitSeconds(0.5)
                .build();
        Action sweepAction4 = drive.actionBuilder(sweepAction4Pose)
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(-54, 38, Math.toRadians(45)), Math.toRadians(-135))
                .waitSeconds(0.5)
                .build();
        Action sweepAction5 = drive.actionBuilder(sweepAction5Pose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-54, 48, Math.toRadians(-45)), Math.toRadians(90))
                .waitSeconds(0.5)
                .build();
        Action sweepAction6 = drive.actionBuilder(sweepAction6Pose)
                .setTangent(0)
                .splineToLinearHeading(new Pose2d(-36, 56, Math.toRadians(-45)), Math.toRadians(0))
                .waitSeconds(0.5)
                .build();

        Action action1 = drive.actionBuilder(action1Pose)
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(-5, 40, Math.toRadians(-90)), Math.toRadians(-45))
                .waitSeconds(0.1)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-5, 34, Math.toRadians(-90)), Math.toRadians(-90))
                .waitSeconds(0.5)
                .build();
        Action action2 = drive.actionBuilder(action2Pose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-36, 56, Math.toRadians(-45)), Math.toRadians(135))
                .waitSeconds(0.5)
                .build();
        Action action3 = drive.actionBuilder(action3Pose)
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(-5, 40, Math.toRadians(-90)), Math.toRadians(-45))
                .waitSeconds(0.1)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-5, 34, Math.toRadians(-90)), Math.toRadians(-90))
                .waitSeconds(0.5)
                .build();
        Action action4 = drive.actionBuilder(action4Pose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-36, 56, Math.toRadians(-45)), Math.toRadians(135))
                .waitSeconds(0.5)
                .build();
        Action action5 = drive.actionBuilder(action5Pose)
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(-5, 40, Math.toRadians(-90)), Math.toRadians(-45))
                .waitSeconds(0.1)
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-5, 34, Math.toRadians(-90)), Math.toRadians(-90))
                .waitSeconds(0.5)
                .build();
        Action action6 = drive.actionBuilder(action6Pose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-36, 56, Math.toRadians(-90)), Math.toRadians(135))
                .waitSeconds(0.5)
                .build();

        if (opModeInInit()) {
            arm.init();
            intake.init();
        }
        waitForStart();
        if (opModeIsActive()) {

            Actions.runBlocking(
                    new SequentialAction(
                            action0,
                            sweepAction0,
                            sweepAction1,
                            sweepAction2,
                            sweepAction3,
                            sweepAction4,
                            sweepAction5,
                            sweepAction6,
                            action1,
                            action2,
                            action3,
                            action4
                            //action5,
                            //action6
                    )
            );
        }
    }
}
