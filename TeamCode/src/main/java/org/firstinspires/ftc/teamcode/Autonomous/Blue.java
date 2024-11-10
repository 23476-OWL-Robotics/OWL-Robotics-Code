package org.firstinspires.ftc.teamcode.Autonomous;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utilities.Auto.Arm;
import org.firstinspires.ftc.teamcode.Utilities.Auto.BlueTrajectories;
import org.firstinspires.ftc.teamcode.Utilities.Auto.Intake;
import org.firstinspires.ftc.teamcode.Utilities.Auto.PoseUpdate;

@Autonomous
public class Blue extends LinearOpMode {

    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(36, 60, Math.toRadians(-90)));
        BlueTrajectories trajectories = new BlueTrajectories(drive);
        Arm arm = new Arm(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        PoseUpdate pose = new PoseUpdate(drive);

        waitForStart();
        if (opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(

                    )
            );
        }
    }
}
