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
    Servo armPivot;
    Servo sampleServo;

    // Create PIDF Controller and Arm Parameters
    ArmControllerParams armControllerParams = new ArmControllerParams();
    PIDF_Controller controller;

    boolean run = true;
    double armPosition = 0;

    // Arm Constructor
    public Arm(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armPivot = hardwareMap.get(Servo.class, "armPivot");
        sampleServo = hardwareMap.get(Servo.class, "sampleServo");

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        controller = new PIDF_Controller.Builder()
                .setControllerMotor(armMotor)
                .setControllerParams(armControllerParams.params)
                .setMaxSpeed(1)
                .setStopOnTargetReached(false)
                .setEndPositionError(5)
                .build();
    }

    // Create init() for Auto
    public void init() {
        armPivot.setPosition(0.6);
        sampleServo.setPosition(0.81);
    }

    public class RunController implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            if (!run) {
                return false;
            } else {
                controller.loopController();
                controller.runController(true);
                return true;
            }
        }
    }
    public Action runController() {
        return new RunController();
    }

    public class StopController implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            run = false;
            return false;
        }
    }
    public Action stopController() {
        return new StopController();
    }

    // The ArmUp class and action extends the arm slides to the height of the Low Basket
    public class ArmUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket p) {

            if (!initialized) {
                armPosition = 23;
                controller.extendTo(armPosition);
                initialized = true;
            }

            double pos = armMotor.getCurrentPosition();
            p.put("Motor Position: ", pos);
            return !controller.targetReached;
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
                armPosition = 37;
                controller.extendTo(armPosition);
                initialized = true;
            }

            double pos = armMotor.getCurrentPosition();
            p.put("Motor Position: ", pos);
            if (controller.targetReached) {
                return false;
            } else {
                controller.loopController();
                return true;
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
                armPosition = 13.3;
                controller.extendTo(armPosition);
                initialized = true;
            }

            double pos = armMotor.getCurrentPosition();
            p.put("Motor Position: ", pos);
            if (controller.targetReached) {
                return false;
            } else {
                controller.loopController();
                return true;
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
                armPosition = 0;
                controller.extendTo(armPosition);
                initialized = true;
            }

            double pos = armMotor.getCurrentPosition();
            p.put("Motor Position: ", pos);
            if (controller.targetReached) {
                return false;
            } else {
                controller.loopController();
                return true;
            }
        }
    }
    public Action armDown() {
        return new ArmDown();
    }

    // Releases the Specimen
    public class Release implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket p) {

            sampleServo.setPosition(0.65);
            return false;
        }
    }
    public Action release() {
        return new Release();
    }

    public class PivotArm implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket p) {

            if (armPivot.getPosition() >= 0.5) {
                armPivot.setPosition(0.1);
            } else if (armPivot.getPosition() < 0.5) {
                armPivot.setPosition(0.8);
            }
            try{TimeUnit.MILLISECONDS.sleep(500);}
            catch (InterruptedException e){
                //nothing
            }
            return false;
        }
    }
    public Action pivotArm() {
        return new PivotArm();
    }

    public class WaitTime implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            try {
                TimeUnit.MILLISECONDS.sleep(800);
            } catch (InterruptedException e) {
                //Nothing
            }
            return false;
        }
    }
    public Action waitTme() {
        return new WaitTime();
    }

}
