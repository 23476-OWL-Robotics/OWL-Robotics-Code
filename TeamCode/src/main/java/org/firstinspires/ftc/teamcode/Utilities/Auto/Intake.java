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

import java.util.Scanner;
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
    Servo sampleServo;

    // Create distanceSensor
    DistanceSensor distanceSensor;

    // Variables for Intake
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
        sampleServo = hardwareMap.get(Servo.class, "sampleServo");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "blockDet");

        right.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        controller = new PIDF_Controller.Builder()
                .setControllerMotor(intakeMotor)
                .setControllerParams(intakeControllerParams.params)
                .setMaxSpeed(1)
                .setStopOnTargetReached(true)
                .setEndPositionError(15)
                .build();
    }

    // Create init() for Auto
    public void init() {
        intakePivot.setPosition(0.5);
    }

    // The IntakeOut class & action tell the intake slide to fully extend
    // The maximum intake extension is 19in
    public class IntakeOut implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket p) {

            if (!initialized) {
                controller.extendTo(19);
                initialized = true;
            }

            double pos = intakeMotor.getCurrentPosition();
            p.put("Motor Position: ", pos);
            if (controller.running) {
                controller.loopController();
                return true;
            } else {
                return false;
            }
        }
    }
    public Action intakeOut() {
        return new IntakeOut();
    }

    // The IntakeIn class and action returns the intake slide to the 0 position (0in)
    public class IntakeIn implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket p) {

            if (!initialized) {
                controller.retractTo(0.2);
                initialized = true;

            }

            double pos = intakeMotor.getCurrentPosition();
            p.put("Motor Position: ", pos);
            if (controller.targetReached) {
                sampleServo.setPosition(0.65);
                return false;

            } else {
                controller.loopController();
                intakePivot.setPosition(0.73);
                return true;
            }
        }
    }
    public Action intakeIn() {
        return new IntakeIn();
    }

    // Intakes a sample
    public class IntakeSample implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            if (!initialized) {
                intakePivot.setPosition(0.12);
                left.setPower(1);
                right.setPower(1);
                controller.extendTo(14);
                initialized = true;
                sampleDetected = false;
            }

            if (sampleDetected || controller.targetReached) {
                intakePivot.setPosition(0.5);
                left.setPower(0);
                right.setPower(0);
                intakeMotor.setPower(0);
                return false;
            } else {
                controller.loopController();
                SampleDetection();
                return true;
            }
        }
    }
    public Action intakeSample() {
        return new IntakeSample();
    }

    public class IntakeSampleSlow implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            if (!initialized) {
                intakePivot.setPosition(0.12);
                left.setPower(1);
                right.setPower(1);
                controller.setMaxSpeed(0.3);
                controller.extendTo(6);
                initialized = true;
                sampleDetected = false;
            }

            if (sampleDetected || controller.targetReached) {
                intakePivot.setPosition(0.5);
                left.setPower(0);
                right.setPower(0);
                intakeMotor.setPower(0);
                return false;
            } else {
                controller.loopController();
                SampleDetection();
                return true;
            }
        }
    }
    public Action intakeSampleSlow() {
        return new IntakeSampleSlow();
    }

    public class OutputSample implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intakePivot.setPosition(0.4);
            left.setPower(-1);
            right.setPower(-1);
            try {
                TimeUnit.MILLISECONDS.sleep(100);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            left.setPower(0);
            right.setPower(0);
            intakePivot.setPosition(0.5);
            return false;
        }
    }

    // Transfers the sample
    public class TransferSample implements Action {
        private boolean initialized = false;
        TransferThread transferThread = new TransferThread();
        Thread thread = new Thread(transferThread);


        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            if (!initialized) {
                thread.start();
                initialized = true;
            }
            return thread.getState() != Thread.State.TERMINATED;
        }
    }
    public Action transferSample() {
        return new TransferSample();
    }

    public class ToEndPositions implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            intakePivot.setPosition(0.71);
            return false;
        }
    }
    public Action toEndPositions() {
        return new ToEndPositions();
    }

    public boolean threadRunning = false;
    class TransferThread implements Runnable {

        @Override
        public void run() {
            try {
                threadRunning = true;
                sampleServo.setPosition(0.7);
                intakePivot.setPosition(0.71);
                TimeUnit.MILLISECONDS.sleep(50);
                left.setPower(-0.3);
                right.setPower(-0.3);
                TimeUnit.MILLISECONDS.sleep(200);
                sampleServo.setPosition(0.770);
                left.setPower(0);
                right.setPower(0);
                TimeUnit.MILLISECONDS.sleep(300);
                intakePivot.setPosition(0.5);
                threadRunning = false;
            } catch (InterruptedException e) {
                // Nothing
            }
        }
    }

    // Sample detection is required to tell if the robot has a sample
    public void SampleDetection() {
        if (!sampleDetected) {
            if (distanceSensor.getDistance(DistanceUnit.CM) < 4.5) {
                sampleDetected = true;
            }
        } else {
            sampleDetected = false;
        }
    }
}
