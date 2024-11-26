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
import org.firstinspires.ftc.teamcode.Utilities.Auto.Intake;
import org.firstinspires.ftc.teamcode.Utilities.Auto.PoseUpdate;
import org.firstinspires.ftc.teamcode.Utilities.Auto.RedTrajectories;

@Config
@Autonomous
public class Red extends LinearOpMode {

    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-36 ,-60, Math.toRadians(90)));
        RedTrajectories trajectories = new RedTrajectories(drive);
        Arm arm = new Arm(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        PoseUpdate pose = new PoseUpdate(drive);

        while (opModeInInit()) {
            arm.init();
            intake.init();
        }
        waitForStart();
        if (opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(

                    )
            );
        }
    }
}
