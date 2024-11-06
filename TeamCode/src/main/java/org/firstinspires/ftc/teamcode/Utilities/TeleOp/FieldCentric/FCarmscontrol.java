package org.firstinspires.ftc.teamcode.Utilities.TeleOp.FieldCentric;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class FCarmscontrol extends FCdrivecontrol {
    public int intakeWristPos;


    float spinnerStick;
    float intakeStick;



    DcMotorEx armMotor;
    DcMotorEx intakeMotor;


    Servo specimenClaw;
    Servo intakePivot;

    CRServo portIntake;
    CRServo starbordIntake;

    DistanceSensor blockDet_DistanceSensor;
    ColorSensor blockDet;
    public void armControl(){



        if (Math.abs(gamepad2.left_stick_y) > 0.05) {
            intakeWristPos += 0.01 * intakeStick;
        }

        //intake servo conrtol
        portIntake.setPower(spinnerStick);
        starbordIntake.setPower(spinnerStick);
        intakePivot.setPosition(intakeWristPos);


        // lifter viper slide control
        if (gamepad2.dpad_up) {
            armMotor.setPower(0.7);
        } else if (gamepad2.dpad_down) {
            armMotor.setPower(-0.7);
        } else {
            armMotor.setPower(0);
        }

        //intake viper slide control
        if (gamepad2.dpad_left) {
            intakeMotor.setPower(0.7);
        } else if (gamepad2.dpad_right) {
            intakeMotor.setPower(-0.7);
        } else intakeMotor.setPower(0);

        //specimin claw control
        if (gamepad2.x) {
            specimenClaw.setPosition(0.75);
        } else if (gamepad2.b) {
            specimenClaw.setPosition(1);
        }
    }

    //setup the arm motors
    public void armMotorSetup(){
        arms.portIntake.setDirection(CRServo.Direction.REVERSE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
