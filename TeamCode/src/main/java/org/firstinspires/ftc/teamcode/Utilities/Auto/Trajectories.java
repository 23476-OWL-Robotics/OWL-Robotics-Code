package org.firstinspires.ftc.teamcode.Utilities.Auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

public class Trajectories extends LinearOpMode {

    @Override
    public void runOpMode() {

    }

    MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0 ,0, Math.toRadians(0)));

    Pose2d traj1Pose = new Pose2d(0, 0, Math.toRadians(0));

    TrajectoryActionBuilder traj1 = drive.actionBuilder(traj1Pose)
            .waitSeconds(3);

    public Action action1 = traj1.build();
}
