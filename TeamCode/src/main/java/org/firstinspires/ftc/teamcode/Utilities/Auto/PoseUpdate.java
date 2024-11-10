package org.firstinspires.ftc.teamcode.Utilities.Auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

public class PoseUpdate {

    MecanumDrive drive;

    public class UpdatePose implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            drive.updatePoseEstimate();
            return false;
        }
    }
    public Action updatePose(MecanumDrive drive) {
        this.drive = drive;
        return new UpdatePose();
    }
}
