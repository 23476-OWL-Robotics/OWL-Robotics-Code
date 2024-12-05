package org.firstinspires.ftc.teamcode.Utilities.TeleOp.RoadRunner;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

public class Trajectories {

    MecanumDrive drive;

    public Trajectories(MecanumDrive drive) {
        this.drive = drive;
    }

    Pose2d startPose = new Pose2d(36, 60, Math.toRadians(-90));

    Action action1 = drive.actionBuilder(startPose)
            .build();
    Action action2 = drive.actionBuilder(drive.pose)
            .build();
}
