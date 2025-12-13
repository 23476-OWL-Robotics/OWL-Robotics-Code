package org.firstinspires.ftc.teamcode.Util.Mechanisms;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lifter {

    TelemetryManager telemetry;
    OpMode opMode;

    Gamepad gamepad1;
    Gamepad gamepad2;

    HardwareMap hardwareMap;

    DcMotorEx leftLiferMotor;
    DcMotorEx rightLifterMotor;

    public Lifter(OpMode opMode, TelemetryManager m) {
        this.opMode = opMode;
        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = m;
    }

    public void init() {
        leftLiferMotor = hardwareMap.get(DcMotorEx.class, "leftLifterMotor");
        rightLifterMotor = hardwareMap.get(DcMotorEx.class, "rightLifterMotor");
    }
}
