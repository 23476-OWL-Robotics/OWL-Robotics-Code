package org.firstinspires.ftc.teamcode.Utilities.TeleOp.RoadRunner;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

public class Trajectories {

    MecanumDrive drive;

    public Trajectories(MecanumDrive drive) {
        this.drive = drive;
    }

    TrajectoryActionBuilder trajectory1 = drive.actionBuilder(drive.pose)
            .waitSeconds(2);

    Action action1 = trajectory1.fresh()
            .build();
}
