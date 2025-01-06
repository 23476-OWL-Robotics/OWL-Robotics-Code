package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Drive")
@Disabled
public class TwoWheelDrive extends LinearOpMode {

    DcMotorEx leftMotor;
    DcMotorEx rightMotor;

    double x, y;
    double speedModifier = 0.3;

    @Override
    public void runOpMode() {

        leftMotor = hardwareMap.get(DcMotorEx.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightMotor");

        waitForStart();
        if (opModeIsActive()) {

            rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            while (opModeIsActive()) {

                x = gamepad1.right_stick_x;
                y = gamepad1.left_stick_y;

                if (y > 0.05 || y < -0.05 || x > 0.05 || x < -0.05) {
                    leftMotor.setPower((y + x) * speedModifier);
                    rightMotor.setPower((y - x) * speedModifier);
                } else {
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);
                }

                if (gamepad2.dpad_up ) {
                    speedModifier ++;
                    try {
                        TimeUnit.MILLISECONDS.sleep(200);
                    } catch (InterruptedException e) {
                        // Nothing
                    }
                } else if (gamepad2.dpad_down) {
                    speedModifier --;
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