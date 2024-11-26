package org.firstinspires.ftc.teamcode.TeleOp.Other;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.TimeUnit;

@TeleOp
public class RobotCentric extends LinearOpMode {

    DcMotorEx frontLeftMotor;
    DcMotorEx frontRightMotor;
    DcMotorEx backLeftMotor;
    DcMotorEx backRightMotor;

    DcMotorEx armMotor;
    DcMotorEx intakeMotor;
    DcMotorEx leftAssentMotor;
    DcMotorEx rightAssentMotor;

    Servo specimenClaw;
    Servo intakePivot;
    Servo sampleServo;
    CRServo left;
    CRServo right;

    ColorSensor colorSensor;
    DistanceSensor distanceSensor;

    double x;
    double y;
    double ls;
    double rs;

    double strafeCorrection = 0.9;
    boolean intake = false;

    double GoBildaServoDownValue = 0.52;

    @Override
    public void runOpMode() throws InterruptedException {

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        leftAssentMotor = hardwareMap.get(DcMotorEx.class, "leftAssentMotor");
        rightAssentMotor = hardwareMap.get(DcMotorEx.class, "rightAssentMotor");

        specimenClaw = hardwareMap.get(Servo.class, "specimenClaw");
        left = hardwareMap.get(CRServo.class, "left");
        right = hardwareMap.get(CRServo.class, "right");
        //colorSensor = hardwareMap.get(ColorSensor.class, "sensor");
        //distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor");
        intakePivot = hardwareMap.get(Servo.class, "intakePivot");
        sampleServo = hardwareMap.get(Servo.class, "sampleServo");

        //ArmControllerParams armControllerParams = new ArmControllerParams();
        //IntakeControllerParams intakeControllerParams = new IntakeControllerParams();
        //LeftAssentControllerParams leftAssentControllerParams = new LeftAssentControllerParams();
        //RightAssentControllerParams rightAssentControllerParams = new RightAssentControllerParams();

        //PIDF_Controller armController = new PIDF_Controller(armControllerParams.params, armMotor);
        //PIDF_Controller intakeController = new PIDF_Controller(intakeControllerParams.params, intakeMotor);
        //PIDF_Controller leftAssentController = new PIDF_Controller(leftAssentControllerParams.params, leftAssentMotor);
        //PIDF_Controller rightAssentController = new PIDF_Controller(rightAssentControllerParams.params, rightAssentMotor);

        if (opModeInInit()) {
            intakePivot.setDirection(Servo.Direction.REVERSE);
            while (opModeInInit()) {
                intakePivot.setPosition(0.5);
                sampleServo.setPosition(0.6);
            }
        }

        waitForStart();
        if (opModeIsActive()) {

            frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            rightAssentMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            right.setDirection(DcMotorSimple.Direction.REVERSE);
            //intakePivot.setDirection(Servo.Direction.REVERSE);

            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftAssentMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightAssentMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftAssentMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightAssentMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            while (opModeIsActive()) {

                x = gamepad1.right_stick_x;
                y = gamepad1.left_stick_y;
                ls = gamepad1.left_trigger;
                rs = gamepad1.right_trigger;

                if (gamepad1.right_bumper) {
                    if (y > 0.05 || y < -0.05 || x > 0.05 || x < -0.05) {
                        frontLeftMotor.setPower((y + x) * 0.3);
                        frontRightMotor.setPower((y - x) * 0.3);
                        backLeftMotor.setPower((y + x) * 0.3);
                        backRightMotor.setPower((y - x) * 0.3);
                    } else if (ls > 0.2 || rs > 0.1) {
                        frontLeftMotor.setPower(((ls - rs) * 0.3) * strafeCorrection);
                        frontRightMotor.setPower(((-ls + rs) * 0.3) * strafeCorrection);
                        backLeftMotor.setPower((-ls + rs) * 0.3);
                        backRightMotor.setPower((ls - rs) * 0.3);
                    } else {
                        frontLeftMotor.setPower(0);
                        frontRightMotor.setPower(0);
                        backLeftMotor.setPower(0);
                        backRightMotor.setPower(0);
                    }
                } else {
                    if (y > 0.05 || y < -0.05 || x > 0.05 || x < -0.05) {
                        frontLeftMotor.setPower(y + x);
                        frontRightMotor.setPower(y - x);
                        backLeftMotor.setPower(y + x);
                        backRightMotor.setPower(y - x);
                    } else if (ls > 0.2 || rs > 0.1) {
                        frontLeftMotor.setPower(ls - rs);
                        frontRightMotor.setPower(-ls + rs);
                        backLeftMotor.setPower((-ls + rs) * strafeCorrection);
                        backRightMotor.setPower((ls - rs) * strafeCorrection);
                    } else {
                        frontLeftMotor.setPower(0);
                        frontRightMotor.setPower(0);
                        backLeftMotor.setPower(0);
                        backRightMotor.setPower(0);
                    }
                }

                // Accent Buttons
                if (gamepad2.right_bumper) {
                    leftAssentMotor.setPower(1);
                    rightAssentMotor.setPower(1);
                } else if (gamepad2.left_bumper) {
                    leftAssentMotor.setPower(-1);
                    rightAssentMotor.setPower(-1);
                } else {
                    leftAssentMotor.setPower(0);
                    rightAssentMotor.setPower(0);
                }

                // Arm Slide Buttons
                if (gamepad2.right_stick_y > 0.2) {
                    armMotor.setPower(-gamepad2.right_stick_y);
                } else if (gamepad2.right_stick_y < -0.2) {
                    armMotor.setPower(-gamepad2.right_stick_y);
                } else {
                    if (armMotor.getCurrentPosition() > 500) {
                        armMotor.setPower(0.005);
                    } else {
                        armMotor.setPower(0);
                    }
                }

                /*
                // Intake Slide Buttons
                if (gamepad2.left_stick_y > 0.2) {
                    intakeMotor.setPower(gamepad2.right_stick_y);
                } else if (gamepad2.left_stick_y < -0.2) {
                    intakeMotor.setPower(gamepad2.right_stick_y);
                } else intakeMotor.setPower(0);
                */

                if (gamepad2.dpad_up) {
                    intakeMotor.setPower(1);
                } else if (gamepad2.dpad_down) {
                    intakeMotor.setPower(-1);
                } else {
                    intakeMotor.setPower(0);
                }

                // Intake Buttons
                if (gamepad2.left_trigger > 0.2) {
                    left.setPower(1);
                    right.setPower(1);
                } else if (gamepad2.right_trigger > 0.2) {
                    left.setPower(-1);
                    right.setPower(-1);
                } else {
                    left.setPower(0);
                    right.setPower(0);
                }

                // Specimen Claw Buttons
                if (gamepad2.x) {
                    specimenClaw.setPosition(0.75);
                } else if (gamepad2.b) {
                    specimenClaw.setPosition(1);
                }


                if (gamepad2.dpad_left) {
                    intakePivot.setPosition(0.5);
                } else if (gamepad2.dpad_right) {
                    intakePivot.setPosition(0.84);
                } else if (gamepad2.left_trigger > 0.2) {
                    left.setPower(1);
                    right.setPower(1);
                } else if (gamepad2.right_trigger > 0.2) {
                    left.setPower(-1);
                    right.setPower(-1);
                } else {
                    if (intakeMotor.getCurrentPosition() < -500) {
                        intakePivot.setPosition(0.8);
                        left.setPower(1);
                        right.setPower(1);
                    } else {
                        intakePivot.setPosition(0.33);
                        left.setPower(0);
                        right.setPower(0);
                    }
                }

                if (gamepad2.a) {
                    sampleServo.setPosition(1);
                } else if (gamepad2.y) {
                    sampleServo.setPosition(0.6);
                }

                //armController.loopController();
                //intakeController.loopController();
                //leftAssentController.loopController();
                //rightAssentController.loopController();

                telemetry.addData("Arm: ", armMotor.getCurrentPosition());
                telemetry.addData("Intake: ", intakeMotor.getCurrentPosition());
                telemetry.addData("Left Assent: ", leftAssentMotor.getCurrentPosition());
                telemetry.addData("Right Assent: ", rightAssentMotor.getCurrentPosition());
                telemetry.addData("Intake Pivot" ,intakePivot.getPosition());
                telemetry.addLine();
                //telemetry.addData("Sensor Distance", distanceSensor.getDistance(DistanceUnit.INCH));
                //telemetry.addData("Color RED", colorSensor.red());
                //telemetry.addData("Color GREEN", colorSensor.green());
                //telemetry.addData("Color BLUE", colorSensor.blue());
                telemetry.update();
            }
        }
    }
}
