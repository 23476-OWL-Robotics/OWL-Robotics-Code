package org.firstinspires.ftc.teamcode.TeleOp.FieldCentric;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class FCarmscontrol extends FCdrivecontrol{
    DcMotorEx armMotor;
    DcMotorEx intakeMotor;
    DcMotorEx leftAssentMotor;
    DcMotorEx rightAssentMotor;

    Servo specimenClaw;
    public void armControl(){
        if (gamepad2.dpad_up) {
            armMotor.setPower(0.7);
        } else if (gamepad2.dpad_down) {
            armMotor.setPower(-0.7);
        } else {
            armMotor.setPower(0);
        }

        if (gamepad2.dpad_left) {
            intakeMotor.setPower(0.7);
        } else if (gamepad2.dpad_right) {
            intakeMotor.setPower(-0.7);
        } else intakeMotor.setPower(0);

        if (gamepad2.x) {
            specimenClaw.setPosition(0.75);
        } else if (gamepad2.b) {
            specimenClaw.setPosition(1);
        }
    }
}
