package org.firstinspires.ftc.teamcode.Utilities.Auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Utilities.PIDF_Controller.ControllerParams.ArmControllerParams;
import org.firstinspires.ftc.teamcode.Utilities.PIDF_Controller.PIDF_Controller;

import java.util.concurrent.TimeUnit;

/*
    This file contains Actions for the robot arm.
    These actions can be called in actions.runBlocking
*/
public class Arm {

    // Create motor and servos
    DcMotorEx armMotor;
    Servo sampleServo;
    Servo specimenServo;

    // Create PIDF Controller and Arm Parameters
    ArmControllerParams armControllerParams = new ArmControllerParams();
    PIDF_Controller controller;

    // Arm Constructor
    public Arm(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        sampleServo = hardwareMap.get(Servo.class, "sampleServo");
        specimenServo = hardwareMap.get(Servo.class, "specimenClaw");

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        controller = new PIDF_Controller(armControllerParams.params, armMotor);
        controller.setMaxSpeed(1);
        controller.setStopOnTargetReached(true);
    }

    // Create init() for Auto
    public void init() {
        sampleServo.setPosition(0.485);
        specimenServo.setPosition(1);
    }

    // The ArmUp class and action extends the arm slides to the height of the Low Basket
    public class ArmUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket p) {

            if (!initialized) {
                controller.extendTo(25);
                initialized = true;
            }

            double pos = armMotor.getCurrentPosition();
            p.put("Motor Position: ", pos);
            if (controller.running) {
                controller.loopController();
                return true;
            } else {
                return false;
            }
        }
    }
    public Action armUp() {
        return new ArmUp();
    }

    // The ArmUpHigh class and action extends the arm slides to the height of the High Basket
    public class ArmUpHigh implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket p) {

            if (!initialized) {
                controller.extendTo(37);
                initialized = true;
            }

            double pos = armMotor.getCurrentPosition();
            p.put("Motor Position: ", pos);
            if (controller.running) {
                controller.loopController();
                return true;
            } else {
                return false;
            }
        }
    }
    public Action armUpHigh() {
        return new ArmUpHigh();
    }

    // The ArmUpSpecimen class and action extends the arm slides to the height of the High Specimen Bar
    public class ArmUpSpecimen implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket p) {

            if (!initialized) {
                controller.extendTo(22);
                initialized = true;
            }

            double pos = armMotor.getCurrentPosition();
            p.put("Motor Position: ", pos);
            if (controller.running) {
                controller.loopController();
                return true;
            } else {
                return false;
            }
        }
    }
    public Action armUpSpecimen() {
        return new ArmUpSpecimen();
    }

    // The ArmDown class and action lowers the arm slides to the o position (0in)
    public class ArmDown implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket p) {

            if (!initialized) {
                controller.retractTo(0);
                initialized = true;
            }

            double pos = armMotor.getCurrentPosition();
            p.put("Motor Position: ", pos);
            if (controller.running) {
                controller.loopController();
                return true;
            } else {
                return false;
            }
        }
    }
    public Action armDown() {
        return new ArmDown();
    }

    // Releases the Specimen
    public class ReleaseSpecimen implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket p) {

            if (!initialized) {
                controller.retractTo(20);
                initialized = true;
            }

            double pos = armMotor.getCurrentPosition();
            p.put("Motor Position: ", pos);
            if (controller.running) {
                controller.loopController();
                return true;
            } else {
                specimenServo.setPosition(0.73);
                return false;
            }
        }
    }
    public Action releaseSpecimen() {
        return new ReleaseSpecimen();
    }

    // Releases the Sample
    public class ReleaseSample implements Action {

        @Override public boolean run(@NonNull TelemetryPacket p) {
            sampleServo.setPosition(0.25);
            try {
                TimeUnit.MILLISECONDS.sleep(850);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            sampleServo.setPosition(0.485);
            return false;
        }
    }
    public Action releaseSample() {
        return new ReleaseSample();
    }
}
