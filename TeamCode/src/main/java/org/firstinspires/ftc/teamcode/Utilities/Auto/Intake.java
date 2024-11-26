package org.firstinspires.ftc.teamcode.Utilities.Auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Utilities.PIDF_Controller.ControllerParams.IntakeControllerParams;
import org.firstinspires.ftc.teamcode.Utilities.PIDF_Controller.PIDF_Controller;
import org.firstinspires.ftc.teamcode.Utilities.TeleOp.SampleDetection;

import java.util.concurrent.TimeUnit;

/*
    This file contains Actions for the robot intake.
    These actions can be called in actions.runBlocking
 */
public class Intake {

    // Create Motor and Servos
    DcMotorEx intakeMotor;
    CRServo left;
    CRServo right;
    Servo intakePivot;

    // Create distanceSensor
    DistanceSensor distanceSensor;

    // Variables for Intake
    int inches;
    boolean sampleDetected = false;

    // Create PIDF Controller and Intake Parameters
    IntakeControllerParams intakeControllerParams = new IntakeControllerParams();
    PIDF_Controller controller;

    // Intake Constructor
    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        left = hardwareMap.get(CRServo.class, "left");
        right = hardwareMap.get(CRServo.class, "right");
        intakePivot = hardwareMap.get(Servo.class, "intakePivot");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor");

        right.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void init() {
        intakePivot.setPosition(0.5);
    }

    public class IntakeOut implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket p) {

            if (!initialized) {
                controller = new PIDF_Controller(intakeControllerParams.params, intakeMotor);
                controller.setStopOnTargetReached(true);
                controller.extendTo(inches);
                initialized = true;
            }

            double pos = intakeMotor.getCurrentPosition();
            p.put("Motor Position: ", pos);
            if (!controller.running) {
                return true;
            } else {
                controller.loopController();
                return false;
            }
        }
    }
    public Action intakeOut(int inches) {
        this.inches = inches;
        return new IntakeOut();
    }

    public class IntakeIn implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket p) {

            if (!initialized) {
                controller = new PIDF_Controller(intakeControllerParams.params, intakeMotor);
                controller.setStopOnTargetReached(true);
                controller.retractTo(inches);
                initialized = true;
            }

            double pos = intakeMotor.getCurrentPosition();
            p.put("Motor Position: ", pos);
            if (!controller.running) {
                return true;
            } else {
                controller.loopController();
                return false;
            }
        }
    }
    public Action intakeIn(int inches) {
        this.inches = inches;
        return new IntakeIn();
    }

    public class IntakeSample implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket p) {

            if (!initialized) {
                intakePivot.setPosition(0.12);
                left.setPower(1);
                right.setPower(1);
                intakeMotor.setPower(0.1);
                initialized = true;
            }

            if (sampleDetected) {
                intakePivot.setPosition(0.5);
                left.setPower(0);
                right.setPower(0);
                intakeMotor.setPower(0);
                return false;
            } else {
                SampleDetection();
                return true;
            }
        }
    }
    public Action intakeSample() {
        return new IntakeSample();
    }

    public class TransferSample implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket p) {

            if (!initialized) {
                intakePivot.setPosition(0.69);
                left.setPower(-1);
                right.setPower(-1);
                initialized = true;
            }
            try {
                TimeUnit.MILLISECONDS.sleep(100);
            } catch (InterruptedException e) {
                // Nothing
            }
            left.setPower(0);
            right.setPower(0);
            intakePivot.setPosition(0.5);
            return false;
        }
    }
    public Action transferSample() {
        return new TransferSample();
    }

    public void SampleDetection() {
        if (!sampleDetected) {
            if (distanceSensor.getDistance(DistanceUnit.CM) < 1) {
                sampleDetected = true;
            }
        } else {
            sampleDetected = false;
        }
    }
}
