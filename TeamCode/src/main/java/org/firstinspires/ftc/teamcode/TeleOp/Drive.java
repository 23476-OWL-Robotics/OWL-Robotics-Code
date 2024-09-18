package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@TeleOp
public class Drive extends LinearOpMode {

    // Drive Motors
    DcMotorEx frontLeftMotor;
    DcMotorEx frontRightMotor;
    DcMotorEx backLeftMotor;
    DcMotorEx backRightMotor;

    // GamePad doubles
    double x;
    double y;
    double xStrafe;

    // Drive correction doubles
    double slowSpeed = 0.3;
    double strafeCorrection;
    double botHeading;
    double oldBotHeading;

    // Drive correction booleans
    boolean startNewHeading;

    @Override
    public void runOpMode() {

        // Add Mecanum Drive
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));

        // Set IMU Parameters and Name
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        ));
        imu.initialize(parameters);

        // Set motors to Drive Motors
        frontLeftMotor = drive.leftFront;
        frontRightMotor = drive.rightFront;
        backLeftMotor = drive.leftBack;
        backRightMotor = drive.rightBack;

        // Create drive Actions
        Action moveForward;
        Action moveBackward;

        waitForStart();
        while (opModeIsActive()) {

            // Set GamePad values
            y = gamepad1.left_stick_y;
            x = gamepad1.right_stick_x;
            xStrafe = gamepad1.left_stick_x;

            // Set bot heading
            botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Update Pose
            drive.updatePoseEstimate();

            double positionX = drive.pose.position.x;
            double positionY = drive.pose.position.y;
            double heading = drive.pose.heading.toDouble();

            // Drive code
            if (gamepad1.left_bumper) {
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

                if (gamepad1.right_bumper) {
                    if (y > 0.1 || y < -0.1 || x > 0.1 || x < -0.1) {
                        frontRightMotor.setPower((y + x) * slowSpeed);
                        backRightMotor.setPower((y + x) * slowSpeed);
                        frontLeftMotor.setPower((y - x) * slowSpeed);
                        backLeftMotor.setPower((y - x) * slowSpeed);
                    } if (xStrafe > 0.1 || xStrafe < -0.1) {
                        if (startNewHeading) {
                            oldBotHeading = botHeading;
                            startNewHeading = false;
                        }
                        strafeCorrection = ((oldBotHeading - botHeading) * 1);

                        frontRightMotor.setPower((-xStrafe) * slowSpeed);
                        backRightMotor.setPower(((xStrafe) * strafeCorrection) * slowSpeed);
                        frontLeftMotor.setPower((xStrafe) * slowSpeed);
                        backLeftMotor.setPower(((-xStrafe) * strafeCorrection) * slowSpeed);
                    } else {
                        frontRightMotor.setPower(0);
                        backRightMotor.setPower(0);
                        frontLeftMotor.setPower(0);
                        backLeftMotor.setPower(0);

                        startNewHeading = true;
                    }

                } else if (y > 0.1 || y < -0.1 || x > 0.1 || x < -0.1) {
                    frontRightMotor.setPower(y + x);
                    backRightMotor.setPower(y + x);
                    frontLeftMotor.setPower(y - x);
                    backLeftMotor.setPower(y - x);
                } else if (xStrafe > 0.1 || xStrafe < -0.1) {
                    if (startNewHeading) {
                        oldBotHeading = botHeading;
                        startNewHeading = false;
                    }
                    strafeCorrection = ((oldBotHeading - botHeading) * 1);

                    frontRightMotor.setPower(-xStrafe);
                    backRightMotor.setPower((xStrafe) * strafeCorrection);
                    frontLeftMotor.setPower(xStrafe);
                    backLeftMotor.setPower((-xStrafe) * strafeCorrection);
                } else {
                    frontRightMotor.setPower(0);
                    backRightMotor.setPower(0);
                    frontLeftMotor.setPower(0);
                    backLeftMotor.setPower(0);

                    startNewHeading = true;
                }

            } else {
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-y, -xStrafe), -x));
            }

            if (gamepad1.dpad_up) {

                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .splineToLinearHeading(new Pose2d(24, 0, Math.toRadians(0)), Math.toRadians(0))
                                .build()
                );
            } else if (gamepad1.dpad_down) {

                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .splineToLinearHeading(new Pose2d(0, 0, Math.toRadians(0)), Math.toRadians(0))
                                .build()
                );
            }

            // Telemetry for debugging
            telemetry.addData("Pose X: ", positionY);
            telemetry.addData("Pose Y: ", positionX);
            telemetry.addData("Heading: ", heading);
            telemetry.addLine();
            telemetry.addData("Bot Heading: ", botHeading);
            telemetry.addData("Old Bot Heading: ", oldBotHeading);
            telemetry.addData("Strafe Correction: ", strafeCorrection);
            telemetry.update();
        }
    }
}
