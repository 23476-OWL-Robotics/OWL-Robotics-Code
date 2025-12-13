package org.firstinspires.ftc.teamcode.Util.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {

    DcMotorEx intakeMotor;

    Telemetry telemetry;
    OpMode opMode;

    Gamepad gamepad1;
    Gamepad gamepad2;

    HardwareMap hardwareMap;

    final double MotorForwardPower = 1.0;
    final double MotorReversePower = -0.5;
    final double MotorZeroPower = 0.0;

    public Intake(OpMode o) {
        this.opMode = o;
        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
    }

    public void init() {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
    }

    public void startIntake() {
        intakeMotor.setPower(MotorForwardPower);
    }
    public void reverseIntake() {
        intakeMotor.setPower(MotorReversePower);
    }
    public void stopIntake() {
        intakeMotor.setPower(MotorZeroPower);
    }

    public void Telemetry() {
        telemetry.addLine("----Intake Power----");
        telemetry.addData("Motor Power", intakeMotor.getPower());
    }
}
