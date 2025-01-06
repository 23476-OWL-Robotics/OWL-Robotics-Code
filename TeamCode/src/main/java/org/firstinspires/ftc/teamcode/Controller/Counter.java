package org.firstinspires.ftc.teamcode.Controller;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Disabled
public class Counter extends LinearOpMode {

    DcMotorEx motor;

    @Override
    public void runOpMode() {

        // ToDo: Set the motor name to the motor you want to use
        motor = hardwareMap.get(DcMotorEx.class, "motor");

        waitForStart();
        if (opModeIsActive()) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            // ToDo: Set motor direction if needed
            // motor.setDirection(DcMotorSimple.Direction.REVERSE);
            while (opModeIsActive()) {

                if (gamepad1.dpad_up) {
                    motor.setPower(0.5);
                } else if (gamepad1.dpad_down) {
                    motor.setPower(-0.5);
                } else {
                    motor.setPower(0);
                }

                // ToDo:
                //  To calculate the conversion unit, divide the distance traveled by the encoder position
                //  inPerTick = inches traveled / encoder position
                //  rotPerTick = rotations traveled / encoder position
                //  degPErTick = degrees traveled / encoder position
                telemetry.addData("Encoder Position: ", motor.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}