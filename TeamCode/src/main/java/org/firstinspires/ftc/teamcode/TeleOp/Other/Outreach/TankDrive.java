package org.firstinspires.ftc.teamcode.TeleOp.Other.Outreach;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.concurrent.TimeUnit;

@TeleOp
@Disabled
public class TankDrive extends LinearOpMode {

    DcMotorEx leftMotor1;
    DcMotorEx leftMotor2;
    DcMotorEx rightMotor1;
    DcMotorEx rightMotor2;

    double x, y;
    double speedModifier = 0.3;

    @Override
    public void runOpMode() {

        leftMotor1 = hardwareMap.get(DcMotorEx.class, "leftMotor1");
        leftMotor2 = hardwareMap.get(DcMotorEx.class, "leftMotor2");
        rightMotor1 = hardwareMap.get(DcMotorEx.class, "rightMotor1");
        rightMotor2 = hardwareMap.get(DcMotorEx.class, "rightMotor2");

        waitForStart();
        if (opModeIsActive()) {

            leftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            rightMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
            rightMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

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
                telemetry.update();
            }
        }
    }
}
