package org.firstinspires.ftc.teamcode.Utilities.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

public class RedTrajectories {

    MecanumDrive drive;

    public RedTrajectories(MecanumDrive drive) {
        this.drive = drive;
    }

    Pose2d startPose = new Pose2d(-36, -60, Math.toRadians(180));

    TrajectoryActionBuilder trajectory1 = drive.actionBuilder(startPose)
            .setTangent(Math.toRadians(45))
            .splineToLinearHeading(new Pose2d(-12, -36, Math.toRadians(90)), Math.toRadians(45));
    TrajectoryActionBuilder trajectory2 = drive.actionBuilder(drive.pose)
            .setTangent(Math.toRadians(180))
            .splineToLinearHeading(new Pose2d(-48, -48, Math.toRadians(-90)), Math.toRadians(180));
    TrajectoryActionBuilder trajectory3 = drive.actionBuilder(drive.pose)
            .setTangent(Math.toRadians(-135))
            .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(-135)), Math.toRadians(-135));
    TrajectoryActionBuilder trajectory4 = drive.actionBuilder(drive.pose)
            .setTangent(Math.toRadians(90))
            .splineToLinearHeading(new Pose2d(-58, -47, Math.toRadians(-90)), Math.toRadians(90));
    TrajectoryActionBuilder trajectory5 = drive.actionBuilder(drive.pose)
            .setTangent(Math.toRadians(-90))
            .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(-135)), Math.toRadians(-90));
    TrajectoryActionBuilder trajectory6 = drive.actionBuilder(drive.pose)
            .setTangent(Math.toRadians(0))
            .splineToLinearHeading(new Pose2d(47, -47, Math.toRadians(-90)), Math.toRadians(0));
    TrajectoryActionBuilder trajectory7 = drive.actionBuilder(drive.pose)
            .setTangent(Math.toRadians(180))
            .splineToLinearHeading(new Pose2d(-55, -55, Math.toRadians(-135)), Math.toRadians(180));
    TrajectoryActionBuilder trajectory8 = drive.actionBuilder(drive.pose)
            .setTangent(Math.toRadians(90))
            .splineToLinearHeading(new Pose2d(-24, 12, Math.toRadians(180)), Math.toRadians(0));

    public Action action1 = trajectory1.fresh()
            .build();
    public Action action2 = trajectory2.fresh()
            .build();
    public Action action3 = trajectory3.fresh()
            .build();
    public Action action4 = trajectory4.fresh()
            .build();
    public Action action5 = trajectory5.fresh()
            .build();
    public Action action6 = trajectory6.fresh()
            .build();
    public Action action7 = trajectory7.fresh()
            .build();
    public Action action8 = trajectory8.fresh()
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
