package org.firstinspires.ftc.teamcode.Utilities.Auto;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

/*
    This class will update the robot pose in a RR Action.
    To update the pose, call updatePose(drive).
 */
public class PoseUpdate {

    // Create MecanumDrive Instance
    MecanumDrive drive;

    // Create PoseUpdate Constructor
    public PoseUpdate(MecanumDrive drive) {
        this.drive = drive;
    }

    // Create class UpdatePose and Action updatePose
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
