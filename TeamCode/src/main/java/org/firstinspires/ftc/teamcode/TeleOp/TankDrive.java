package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Controller.Controller;
import org.firstinspires.ftc.teamcode.Controller.Params.LeftControllerParams;
import org.firstinspires.ftc.teamcode.Controller.Params.RightControllerParams;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Drive")
public class TankDrive extends LinearOpMode {

    DcMotorEx leftMotor1;
    DcMotorEx leftMotor2;
    DcMotorEx rightMotor1;
    DcMotorEx rightMotor2;

    DcMotorEx leftBarMotor;
    DcMotorEx rightBarMotor;

    Servo left;
    Servo right;

    double x, y;
    double speedModifier = 0.5;

    @Override
    public void runOpMode() {

        leftMotor1 = hardwareMap.get(DcMotorEx.class, "leftMotor1");
        leftMotor2 = hardwareMap.get(DcMotorEx.class, "leftMotor2");
        rightMotor1 = hardwareMap.get(DcMotorEx.class, "rightMotor1");
        rightMotor2 = hardwareMap.get(DcMotorEx.class, "rightMotor2");

        leftBarMotor = hardwareMap.get(DcMotorEx.class, "leftMotor");
        rightBarMotor = hardwareMap.get(DcMotorEx.class, "rightMotor");

        left = hardwareMap.get(Servo.class, "left");
        right = hardwareMap.get(Servo.class, "right");

        waitForStart();
        if (opModeIsActive()) {

            leftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBarMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBarMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            rightMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
            rightMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
            rightBarMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            leftBarMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBarMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            RightControllerParams rightControllerParams = new RightControllerParams();

            Controller rightController = new Controller.Builder()
                    .setControllerMotor(rightBarMotor)
                    .setControllerParams(rightControllerParams.params)
                    .setMaxSpeed(0.2)
                    .setStopOnTargetReached(false)
                    .build();

            while (opModeIsActive()) {

                y = gamepad1.left_stick_y;
                x = gamepad1.right_stick_x;

                if (y > 0.1 || y < -0.1 || x > 0.1 || x < -0.1) {
                    leftMotor1.setPower((y - x) * speedModifier);
                    leftMotor2.setPower((y - x) * speedModifier);
                    rightMotor1.setPower((y + x) * speedModifier);
                    rightMotor2.setPower((y + x) * speedModifier);
                } else {
                    leftMotor1.setPower(0);
                    leftMotor2.setPower(0);
                    rightMotor1.setPower(0);
                    rightMotor2.setPower(0);
                }

                if (gamepad1.a) {
                    left.setPosition(0.8);
                    right.setPosition(0.2);
                } else if (gamepad1.y) {
                    left.setPosition(0.6);
                    right.setPosition(0.4);
                }

                if (gamepad1.dpad_up) {
                    rightController.rotateTo(0);
                } else if (gamepad1.dpad_down) {
                    rightController.rotateTo(60);
                }

                if (gamepad2.dpad_up ) {
                    speedModifier += 0.1;
                    try {
                        TimeUnit.MILLISECONDS.sleep(200);
                    } catch (InterruptedException e) {
                        // Nothing
                    }
                } else if (gamepad2.dpad_down) {
                    speedModifier -= 0.1;
                    try {
                        TimeUnit.MILLISECONDS.sleep(200);
                    } catch (InterruptedException e) {
                        // Nothing
                    }
                }

                rightController.loopController();
                leftBarMotor.setPower(rightController.out);

                telemetry.addData("Speed", speedModifier);
                telemetry.addData("Left Position", leftBarMotor.getCurrentPosition());
                telemetry.addData("Right Position", rightBarMotor.getCurrentPosition());
                telemetry.addLine();
                telemetry.addData("Left Power", rightController.out);
                telemetry.addData("Right Power", rightController.out);
                telemetry.update();
            }
        }
    }
}