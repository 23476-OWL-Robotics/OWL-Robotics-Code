package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utilities.Auto.Arm;
import org.firstinspires.ftc.teamcode.Utilities.Auto.Intake;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
@Disabled
public class BlueSpecimen extends LinearOpMode {

    // Create drive, arm, and intake for rr, arm functions, and intake functions.
    MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-16.5, 62, Math.toRadians(0)));
    Arm arm = new Arm(hardwareMap);
    Intake intake = new Intake(hardwareMap);

    Pose2d action0Pose = new Pose2d(16.5, 62, Math.toRadians(0));
    Pose2d action1Pose = new Pose2d(-10.5, 30, Math.toRadians(-90));
    Pose2d action2Pose = new Pose2d(-36, 62, Math.toRadians(90));
    Pose2d action3Pose = new Pose2d(-10, 34, Math.toRadians(-90));
    Pose2d action4Pose = new Pose2d(-36, 62, Math.toRadians(90));
    Pose2d action5Pose = new Pose2d(-10, 34, Math.toRadians(-90));
    Pose2d action6Pose = new Pose2d(-36, 62, Math.toRadians(90));
    Pose2d action7Pose = new Pose2d(-10, 62, Math.toRadians(-90));

    Action action0 = drive.actionBuilder(action0Pose)
            .waitSeconds(0.5)
            .setTangent(Math.toRadians(-90))
            .splineToSplineHeading(new Pose2d(-10.5, 40, Math.toRadians(-90)), Math.toRadians(-90))
            .waitSeconds(0.1)
            .setTangent(Math.toRadians(-90))
            .splineToLinearHeading(new Pose2d(-10.5, 30, Math.toRadians(-90)), Math.toRadians(-90))
            .build();
    Action action1 = drive.actionBuilder(action1Pose)
            .setTangent(Math.toRadians(-90))
            .splineToLinearHeading(new Pose2d(-10.5, 30, Math.toRadians(-90)), Math.toRadians(-90))
            .setTangent(Math.toRadians(135))
            .splineToSplineHeading(new Pose2d(-26, 34, Math.toRadians(-90)), Math.toRadians(180))
            .splineToLinearHeading(new Pose2d(-46, 0, Math.toRadians(-90)), Math.toRadians(-145))
            .setTangent(Math.toRadians(90))
            .lineToY(52)
            .setTangent(Math.toRadians(-90))
            .lineToYSplineHeading(24, Math.toRadians(-90))
            .splineToLinearHeading(new Pose2d(-56, 12, Math.toRadians(-90)), Math.toRadians(180))
            .setTangent(Math.toRadians(90))
            .lineToY(52)
            .setTangent(Math.toRadians(-90))
            .lineToYSplineHeading(24, Math.toRadians(-90))
            .splineToLinearHeading(new Pose2d(-62, 12, Math.toRadians(-90)), Math.toRadians(180))
            .setTangent(Math.toRadians(90))
            .lineToY(52)
            .setTangent(Math.toRadians(45))
            .splineToLinearHeading(new Pose2d(-36, 62, Math.toRadians(90)), Math.toRadians(45))
            .build();
    Action action2 = drive.actionBuilder(action2Pose)
            .setTangent(Math.toRadians(-45))
            .splineToLinearHeading(new Pose2d(-10, 34, Math.toRadians(-90)), Math.toRadians(-45))
            .build();
    Action action3 = drive.actionBuilder(action3Pose)
            .setTangent(Math.toRadians(135))
            .splineToLinearHeading(new Pose2d(-36, 62, Math.toRadians(90)), Math.toRadians(135))
            .build();
    Action action4 = drive.actionBuilder(action4Pose)
            .setTangent(Math.toRadians(-45))
            .splineToLinearHeading(new Pose2d(-10, 34, Math.toRadians(-90)), Math.toRadians(-45))
            .build();
    Action action5 = drive.actionBuilder(action5Pose)
            .setTangent(Math.toRadians(135))
            .splineToLinearHeading(new Pose2d(-36, 62, Math.toRadians(90)), Math.toRadians(135))
            .build();
    Action action6 = drive.actionBuilder(action6Pose)
            .setTangent(Math.toRadians(-45))
            .splineToLinearHeading(new Pose2d(-10, 34, Math.toRadians(-90)), Math.toRadians(-45))
            .build();
    Action action7 = drive.actionBuilder(action7Pose)
            .setTangent(Math.toRadians(135))
            .splineToLinearHeading(new Pose2d(-36, 62, Math.toRadians(90)), Math.toRadians(135))
            .build();

    @Override
    public void runOpMode() {

        if (opModeInInit()) {
            arm.init();
            intake.init();
        }
        waitForStart();
        if (opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(
                            action0,
                            action1,
                            action2,
                            action3,
                            action4,
                            action5,
                            action6,
                            action7
                    )
            );
        }
    }
}
