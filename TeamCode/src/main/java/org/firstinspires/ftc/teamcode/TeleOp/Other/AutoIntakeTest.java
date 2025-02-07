package org.firstinspires.ftc.teamcode.TeleOp.Other;

import static java.lang.Math.abs;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utilities.TeleOp.AutoBlockDetection;


@TeleOp
@Disabled
public class AutoIntakeTest extends LinearOpMode {

    DcMotorEx intakeMotor;

    Servo intakePivot;
    Servo intakeRotate;
    Servo intakeGrab;

    Servo armPivot;
    Servo armGrab;

    @Override
    public void runOpMode() {

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakePivot = hardwareMap.get(Servo.class, "intakePivot");
        intakeRotate = hardwareMap.get(Servo.class, "left");
        intakeGrab = hardwareMap.get(Servo.class, "right");

        armPivot = hardwareMap.get(Servo.class, "armPivot");
        armGrab = hardwareMap.get(Servo.class, "sampleServo");

        double intakeRotatePosition = 0.675;

        waitForStart();
        if (opModeIsActive()) {
            intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            intakePivot.setPosition(0.76);
            intakeRotate.setPosition(0.675);
            intakeGrab.setPosition(0.89);
            armPivot.setPosition(0.79);
            intakeGrab.setPosition(0.7);

            AutoBlockDetection blockDetection = new AutoBlockDetection.Builder()
                    .setHardWareMap(hardwareMap)
                    .setColorRange(AutoBlockDetection.ColorRange.YELLOW)
                    .setMotor(intakeMotor)
                    .setPivotServo(intakePivot)
                    .setRotateServo(intakeRotate)
                    .setGrabServo(intakeGrab)
                    .build();

            blockDetection.init();

            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0 ,0, Math.toRadians(-90)));

            while (opModeIsActive()) {

                /*
                if (gamepad1.touchpad) {
                    armGrab.setPosition(0.77);
                }

                if (gamepad1.dpad_up) {
                    intakeGrab.setPosition(0.89);
                    intakeRotate.setPosition(0.675);
                    intakeRotatePosition = 0.675;
                    intakePivot.setPosition(0.76);
                } else if (gamepad1.dpad_down) {
                    intakeGrab.setPosition(0.5);
                    intakePivot.setPosition(0.07);
                }

                if (gamepad1.dpad_left && intakeRotatePosition > 0.35 && intakePivot.getPosition() < 0.5) {
                    intakeRotatePosition -= 0.002;
                    intakeRotate.setPosition(intakeRotatePosition);
                } else if (gamepad1.dpad_right && intakeRotatePosition < 1 && intakePivot.getPosition() < 0.5) {
                    intakeRotatePosition += 0.002;
                    intakeRotate.setPosition(intakeRotatePosition);
                }

                if (gamepad1.a && intakePivot.getPosition() < 0.5) {
                    intakeGrab.setPosition(0.89);
                } else if (gamepad1.b && intakePivot.getPosition() < 0.5) {
                    intakeGrab.setPosition(0.5);
                }

                if (abs(gamepad1.right_stick_y) > 0.2) {
                    intakeMotor.setPower(gamepad1.right_stick_y);
                } else {
                    intakeMotor.setPower(0);
                }

                 */

                if (gamepad1.a) {
                    blockDetection.findSample();
                    telemetry.addData("X", blockDetection.robotPosX);
                    telemetry.addData("Y", blockDetection.robotPosY);
                    telemetry.addData("Angle", blockDetection.angle);
                    telemetry.addLine();
                    telemetry.addData("Block X", blockDetection.blockPosX);
                    telemetry.addData("Block Y", blockDetection.blockPosY);
                    telemetry.addData("Block Angle", blockDetection.blockAngle);
                    telemetry.update();
                }
                if (gamepad1.b) {
                    blockDetection.grabSample(drive);
                }
            }
        }
    }
}
