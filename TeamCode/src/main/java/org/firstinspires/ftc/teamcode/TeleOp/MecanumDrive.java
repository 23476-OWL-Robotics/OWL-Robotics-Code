package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Drive")
@Disabled
public class MecanumDrive extends LinearOpMode {

    DcMotorEx frontLeftMotor;
    DcMotorEx frontRightMotor;
    DcMotorEx backLeftMotor;
    DcMotorEx backRightMotor;

    double x, y, ls, rs;
    double speedModifier = 0.5;

    @Override
    public void runOpMode() {

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        waitForStart();
        if (opModeIsActive()) {

            frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            while (opModeIsActive()) {

                x = gamepad1.right_stick_x;
                y = gamepad1.left_stick_y;
                ls = gamepad1.left_trigger;
                rs = gamepad1.right_trigger;

                if (y > 0.1 || y < -0.1 || x > 0.1 || x < 0.1) {
                    frontLeftMotor.setPower((y - x) * speedModifier);
                    frontRightMotor.setPower((y + x) * speedModifier);
                    backLeftMotor.setPower((y - x) * speedModifier);
                    backRightMotor.setPower((y + x) * speedModifier);
                } else {
                    frontLeftMotor.setPower(0);
                    frontRightMotor.setPower(0);
                    backLeftMotor.setPower(0);
                    backRightMotor.setPower(0);
                }

                if (ls > 0.1 || rs > 0.1) {
                    frontLeftMotor.setPower((ls - rs) * speedModifier);
                    frontRightMotor.setPower((-ls + rs) * speedModifier);
                    backLeftMotor.setPower((-ls + rs) * speedModifier);
                    backRightMotor.setPower((ls - rs) * speedModifier);
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

                telemetry.addData("Speed", speedModifier);
                telemetry.addData("Left", ls);
                telemetry.addData("Right", rs);
                telemetry.update();
            }
        }
    }
}