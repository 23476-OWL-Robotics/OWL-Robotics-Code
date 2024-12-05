package org.firstinspires.ftc.teamcode.Autonomous;

// RR-specific imports
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utilities.Auto.Arm;
import org.firstinspires.ftc.teamcode.Utilities.Auto.BlueTrajectories;
import org.firstinspires.ftc.teamcode.Utilities.Auto.Intake;
import org.firstinspires.ftc.teamcode.Utilities.Auto.PoseUpdate;
import org.firstinspires.ftc.teamcode.Utilities.PIDF_Controller.ControllerParams.ArmControllerParams;
import org.firstinspires.ftc.teamcode.Utilities.PIDF_Controller.PIDF_Controller;

import java.util.concurrent.TimeUnit;

@Autonomous
public class Blue extends LinearOpMode {

    @Override
    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(36, 60, Math.toRadians(-90)));
        //BlueTrajectories trajectories = new BlueTrajectories(drive);
        Arm arm = new Arm(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        //PoseUpdate pose = new PoseUpdate(drive);

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
                            arm.armUp(),
                            arm.releaseSample(),
                            new ParallelAction(
                                    arm.armDown(),
                                    intake.intakeSample(),
                                    intake.intakeIn()
                            ),
                            intake.transferSample(),
                            arm.armUp(),
                            arm.releaseSample(),
                            new ParallelAction(
                                    arm.armDown(),
                                    intake.intakeSample(),
                                    intake.intakeIn()
                            ),
                            intake.transferSample(),
                            arm.armUp(),
                            arm.releaseSample(),
                            new ParallelAction(
                                    arm.armDown(),
                                    intake.intakeSample(),
                                    intake.intakeIn()
                            ),
                            intake.transferSample(),
                            arm.armUp(),
                            arm.releaseSample(),
                            arm.armDown()
                    )
            );
        }
    }
}
