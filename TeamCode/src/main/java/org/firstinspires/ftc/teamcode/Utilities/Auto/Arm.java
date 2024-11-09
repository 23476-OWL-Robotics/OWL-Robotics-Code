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

public class Arm {

    DcMotorEx armMotor;
    Servo sampleServo;
    Servo specimenServo;

    int inches;

    ArmControllerParams armControllerParams = new ArmControllerParams();
    PIDF_Controller controller = new PIDF_Controller(armControllerParams.params, armMotor);

    public Arm(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        sampleServo = hardwareMap.get(Servo.class, "sampleServo");
        specimenServo = hardwareMap.get(Servo.class, "specimenServo");

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public class ArmUp implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket p) {

            if (!initialized) {
                controller.extendTo(inches);
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
    public Action ArmUp (int inches) {
        this.inches = inches;
        return new ArmUp();
    }

    public class ArmDown implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket p) {

            if (!initialized) {
                controller.retractTo(inches);
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
    public Action armDown(int inches) {
        this.inches = inches;
        return new ArmDown();
    }

    public class ReleaseSpecimen implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            specimenServo.setPosition(0.75);
            return false;
        }
    }
    public Action releaseSpecimen() {
        return new ReleaseSpecimen();
    }

    public class ReleaseSample implements Action {

        @Override public boolean run(@NonNull TelemetryPacket p) {
            sampleServo.setPosition(0);
            return false;
        }
    }
    public Action releaseSample() {
        return new ReleaseSample();
    }
}
