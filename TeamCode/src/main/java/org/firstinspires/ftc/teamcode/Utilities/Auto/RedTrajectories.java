package org.firstinspires.ftc.teamcode.Utilities.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

/*
    This class contains all of the trajectories for Autonomous when starting on the red side.
    A visual representation can be found in MeepMeepAuto2.
    Full trajectory found at the bottom of the file.
 */
public class RedTrajectories {

    // Create a MecanumDrive instance.
    MecanumDrive drive;

    // Create RedTrajectories Constructor
    public RedTrajectories(MecanumDrive drive) {
        this.drive = drive;
    }

    // Robot starting position.
    Pose2d startPose = new Pose2d(-36, -60, Math.toRadians(90));

    // Trajectories
    Action action1 = drive.actionBuilder(startPose)
            .setTangent(Math.toRadians(45))
            .splineToLinearHeading(new Pose2d(-12, -36, Math.toRadians(90)), Math.toRadians(45))
            .build();
    Action action2 = drive.actionBuilder(drive.pose)
            .setTangent(Math.toRadians(180))
            .splineToLinearHeading(new Pose2d(-48, -48, Math.toRadians(-90)), Math.toRadians(180))
            .build();
    Action action3 = drive.actionBuilder(drive.pose)
            .setTangent(Math.toRadians(-135))
            .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(-135)), Math.toRadians(-135))
            .build();
    Action action4 = drive.actionBuilder(drive.pose)
            .setTangent(Math.toRadians(90))
            .splineToLinearHeading(new Pose2d(-58, -47, Math.toRadians(-90)), Math.toRadians(90))
            .build();
    Action action5 = drive.actionBuilder(drive.pose)
            .setTangent(Math.toRadians(-90))
            .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(-135)), Math.toRadians(-90))
            .build();
    Action action6 = drive.actionBuilder(drive.pose)
            .setTangent(Math.toRadians(0))
            .splineToLinearHeading(new Pose2d(47, -47, Math.toRadians(-90)), Math.toRadians(0))
            .build();
    Action action7 = drive.actionBuilder(drive.pose)
            .setTangent(Math.toRadians(180))
            .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(-135)), Math.toRadians(180))
            .build();
    Action action8 = drive.actionBuilder(drive.pose)
            .setTangent(Math.toRadians(90))
            .splineToLinearHeading(new Pose2d(-24, 12, Math.toRadians(180)), Math.toRadians(0))
            .build();


    /*
            .setTangent(Math.toRadians(45))
            .splineToLinearHeading(new Pose2d(-12, -36, Math.toRadians(90)), Math.toRadians(45))
            .waitSeconds(2)
            .setTangent(Math.toRadians(180))
            .splineToLinearHeading(new Pose2d(-48, -48, Math.toRadians(-90)), Math.toRadians(180))
            .waitSeconds(2)
            .setTangent(Math.toRadians(-135))
            .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(-135)), Math.toRadians(-135))
            .waitSeconds(1)
            .setTangent(Math.toRadians(90))
            .splineToLinearHeading(new Pose2d(-58, -47, Math.toRadians(-90)), Math.toRadians(90))
            .waitSeconds(2)
            .setTangent(Math.toRadians(-90))
            .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(-135)), Math.toRadians(-90))
            .waitSeconds(1)
            .setTangent(Math.toRadians(0))
            .splineToLinearHeading(new Pose2d(47, -47, Math.toRadians(-90)), Math.toRadians(0))
            .waitSeconds(2)
            .setTangent(Math.toRadians(180))
            .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(-135)), Math.toRadians(180))
            .waitSeconds(1)
            .setTangent(Math.toRadians(90))
            .splineToLinearHeading(new Pose2d(-24, 12, Math.toRadians(180)), Math.toRadians(0));
    */
}
