package org.firstinspires.ftc.teamcode.Util.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util.PIDFController.Coefficients;
import org.firstinspires.ftc.teamcode.Util.PIDFController.ControllerStates;
import org.firstinspires.ftc.teamcode.Util.PIDFController.VelocityController;

public class Intake {

    DcMotorEx intakeMotor;

    VelocityController vController;

    Telemetry telemetry;
    OpMode opMode;

    HardwareMap hardwareMap;

    final int IntakeRPM = 450;
    final int ReverseRPM = -500;
    final int StopRPM = 0;

    boolean stop = false;

    public Intake(OpMode o) {
        this.opMode = o;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
    }

    public void init() {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        vController = new VelocityController.Builder()
                .setCoefficients(new Coefficients.VelocityCoefficients.IntakeMotorCoefficients())
                .setEndState(ControllerStates.RUN_CONTROLLER)
                .setEndVelocityError(50)
                .build();

        vController.setState(ControllerStates.STOP_CONTROLLER);
    }

    public void loop() {
        vController.runController(intakeMotor.getVelocity());

        if (!stop) {
            intakeMotor.setPower(vController.getOut());
        } else {
            intakeMotor.setPower(StopRPM);
        }
    }

    public void startIntake() {
        vController.setTargetRPM(IntakeRPM);
        vController.setState(ControllerStates.RUN_CONTROLLER);
        stop = false;
    }
    public void reverseIntake() {
        vController.setTargetRPM(ReverseRPM);
        vController.setState(ControllerStates.RUN_CONTROLLER);
        stop = false;
    }
    public void stopIntake() {
        vController.setTargetRPM(StopRPM);
        vController.setState(ControllerStates.RUN_CONTROLLER);
        stop = true;
    }

    public void Telemetry() {
        telemetry.addLine("----Intake----");
        telemetry.addData("Motor Power", intakeMotor.getPower());
        telemetry.addData("Motor Speed", vController.getCurrentRPM());
        telemetry.addData("Controller State", vController.getState());
    }
}
