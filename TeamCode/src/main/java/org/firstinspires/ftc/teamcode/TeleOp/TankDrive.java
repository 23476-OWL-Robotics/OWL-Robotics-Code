package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Controller.Controller;
import org.firstinspires.ftc.teamcode.Controller.Params.LeftControllerParams;
import org.firstinspires.ftc.teamcode.Controller.Params.RightControllerParams;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Drive")
@Disabled
public class TankDrive extends LinearOpMode {

    DcMotorEx leftMotor1;
    DcMotorEx leftMotor2;
    DcMotorEx rightMotor1;
    DcMotorEx rightMotor2;

    DcMotorEx leftBarMotor;
    DcMotorEx rightBarMotor;

    double x, y;
    double speedModifier = 0.3;

    @Override
    public void runOpMode() {

        leftMotor1 = hardwareMap.get(DcMotorEx.class, "leftMotor1");
        leftMotor2 = hardwareMap.get(DcMotorEx.class, "leftMotor2");
        rightMotor1 = hardwareMap.get(DcMotorEx.class, "rightMotor1");
        rightMotor2 = hardwareMap.get(DcMotorEx.class, "rightMotor2");

        leftBarMotor = hardwareMap.get(DcMotorEx.class, "leftMotor");
        rightBarMotor = hardwareMap.get(DcMotorEx.class, "rightMotor");

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

            LeftControllerParams leftControllerParams = new LeftControllerParams();
            RightControllerParams rightControllerParams = new RightControllerParams();

            Controller leftController = new Controller.Builder()
                    .setControllerMotor(leftBarMotor)
                    .setControllerParams(leftControllerParams.params)
                    .setMaxSpeed(0.8)
                    .setStopOnTargetReached(false)
                    .build();

            Controller rightController = new Controller.Builder()
                    .setControllerMotor(leftBarMotor)
                    .setControllerParams(rightControllerParams.params)
                    .setMaxSpeed(0.8)
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

                if (gamepad1.dpad_left) {
                    leftController.rotateTo(0);
                    rightController.rotateTo(0);
                } else if (gamepad1.dpad_right) {
                    leftController.rotateTo(100);
                    rightController.rotateTo(100);
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

                leftController.loopController();
                rightController.loopController();

                telemetry.addData("Speed", speedModifier);
                telemetry.update();
            }
        }
    }
}