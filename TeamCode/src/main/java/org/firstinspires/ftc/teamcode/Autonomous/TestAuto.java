package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utilities.Auto.Arm;
import org.firstinspires.ftc.teamcode.Utilities.Auto.Intake;

@Autonomous
public class TestAuto extends LinearOpMode {

    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));
        Arm arm = new Arm(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        while (opModeInInit()) {
            arm.init();
            intake.init();
        }
        waitForStart();
        if (opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(
                            intake.intakeSample(),
                            intake.intakeIn(),
                            intake.transferSample(),
                            new ParallelAction(
                                    arm.armUpHigh(),
                                    arm.pivotArm()
                            ),
                            arm.release(),
                            new ParallelAction(
                                    arm.pivotArm(),
                                    arm.armDown()
                            )
                    )
            );
        }
    }
}
