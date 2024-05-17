package org.firstinspires.ftc.teamcode.RoadRunner.Tests.TeleOp;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;

@TeleOp
public class LocationDrive extends LinearOpMode {

    public void runOpMode() {

        // Add Mecanum Drive for location tracking
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));

        // Create a new Pose2D for location tracking
        Pose2d robotPose = drive.pose;

        // Create Location doubles
        double positionX;
        double positionY;
        double headingReal;
        double headingImag;

        // Create voltage double
        double voltage;

        // Create GamePad JoyStick X and Y doubles
        double x;
        double y;
        double speed = 0.1;

        waitForStart();
        while (opModeIsActive()) {

            // Set GamePad JoyStick doubles
            x = gamepad1.right_stick_x;
            y = gamepad1.left_stick_y;

            // Set Location doubles
            positionX = robotPose.position.x;
            positionY = robotPose.position.y;
            headingReal = robotPose.heading.real;
            headingImag = robotPose.heading.imag;

            // Set voltage double
            voltage = drive.voltageSensor.getVoltage();

            // Drive code for Robot
            // The power is purposefully low to get good accuracy with the dead wheels.
            if (y > 0.2 || y < -0.2 || x > 0.2 || x < -0.2) {
                drive.leftFront.setPower((y + x) * speed);
                drive.rightFront.setPower((y - x) * speed);
                drive.leftBack.setPower((y + x) * speed);
                drive.rightBack.setPower((y - x) * speed);
            } else if (gamepad1.left_bumper) {
                drive.leftFront.setPower(-speed);
                drive.rightFront.setPower(speed);
                drive.leftBack.setPower(speed);
                drive.rightBack.setPower(-speed);
            } else if (gamepad1.right_bumper) {
                drive.leftFront.setPower(speed);
                drive.rightFront.setPower(-speed);
                drive.leftBack.setPower(-speed);
                drive.rightBack.setPower(speed);
            } else {
                drive.leftFront.setPower(0);
                drive.rightFront.setPower(0);
                drive.leftBack.setPower(0);
                drive.rightBack.setPower(0);
            }

            if (gamepad1.dpad_up) {
                speed += 0.1;
            } else if (gamepad1.dpad_down) {
                speed -= 0.1;
            }

            // Telemetry for positioning
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
