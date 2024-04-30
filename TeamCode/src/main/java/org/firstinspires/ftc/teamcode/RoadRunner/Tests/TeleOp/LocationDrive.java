package org.firstinspires.ftc.teamcode.RoadRunner.Tests.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@TeleOp
public class LocationDrive extends LinearOpMode {

    public void runOpMode() {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));

        Pose2d robotPose = drive.pose;

        double positionX;
        double positionY;
        double headingReal;
        double headingImag;

        double voltage;
        double x;
        double y;

        waitForStart();
        while (opModeIsActive()) {

            x = gamepad1.right_stick_x;
            y = gamepad1.left_stick_y;

            positionX = robotPose.position.x;
            positionY = robotPose.position.y;

            headingReal = robotPose.heading.real;
            headingImag = robotPose.heading.imag;

            voltage = drive.voltageSensor.getVoltage();

            if (y > 0.2 || y < -0.2 || x > 0.2 || x < -0.2) {
                drive.leftFront.setPower((y + x) * 0.1);
                drive.rightFront.setPower((y - x) * 0.1);
                drive.leftBack.setPower((y + x) * 0.1);
                drive.rightBack.setPower((y - x) * 0.1);
            } else if (gamepad1.left_bumper) {
                drive.leftFront.setPower(-0.1);
                drive.rightFront.setPower(0.1);
                drive.leftBack.setPower(0.1);
                drive.rightBack.setPower(-0.1);
            } else if (gamepad1.right_bumper) {
                drive.leftFront.setPower(0.1);
                drive.rightFront.setPower(-0.1);
                drive.leftBack.setPower(-0.1);
                drive.rightBack.setPower(0.1);
            } else {
                drive.leftFront.setPower(0);
                drive.rightFront.setPower(0);
                drive.leftBack.setPower(0);
                drive.rightBack.setPower(0);
            }

            telemetry.addData("Position X: ", positionX);
            telemetry.addData("Position Y: ", positionY);
            telemetry.addData("Heading Real: ", headingReal);
            telemetry.addData("Heading Imag:", headingImag);
            telemetry.addLine();
            telemetry.addData("Voltage = ", voltage);
            telemetry.update();
        }
    }
}
