package org.firstinspires.ftc.teamcode.RoadRunner.Tests.Auto;

//Road Runner Imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

//Non Road Runner Imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@Config
@Autonomous
public class Test6 extends LinearOpMode {

    public void runOpMode() {

        //Add Mecanum Drive
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-48, 0, Math.toRadians(90)));

        //Create position & heading
        Vector2d position;
        Rotation2d heading;

        //Create a new Action
        Action testAction;

        //Set what the action will do.
        testAction = drive.actionBuilder(drive.pose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(0, 48, Math.toRadians(0)), Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(48, 0, Math.toRadians(-90)), Math.toRadians(-90))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(0, -48, Math.toRadians(-180)), Math.toRadians(-180))
                .setTangent(Math.toRadians(-180))
                .splineToLinearHeading(new Pose2d(-48, 0, Math.toRadians(90)), Math.toRadians(90))
                .build();

        //Run testAction
        waitForStart();
        if (opModeIsActive()) {

            Actions.runBlocking(
                    new SequentialAction(
                            testAction
                    )
            );

            //Add values to position and heading
            position = drive.pose.position;
            heading = drive.pose.heading;

            //Add telemetry for Position and Heading
            telemetry.addData("Position = ", position);
            telemetry.addData("Heading = ", heading);
            telemetry.update();
        }
    }
}