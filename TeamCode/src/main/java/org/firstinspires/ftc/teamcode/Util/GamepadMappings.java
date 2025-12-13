package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadMappings {

    public Gamepad gamepad1, gamepad2;

    public GamepadMappings (OpMode o) {
        gamepad1 = o.gamepad1;
        gamepad2 = o.gamepad2;
    }

    public double Drive_Stick_Y() {
        return gamepad1.left_stick_y;
    }
    public double Drive_Stick_X() {
        return gamepad1.left_stick_x;
    }
    public double Drive_Stick_Turn() {
        return gamepad1.right_stick_x;
    }
    public double Drive_Slow_Trigger() {
        return gamepad1.right_trigger;
    }

    public boolean Drive_Score_1() {
        return gamepad1.y && gamepad1.dpad_up;
    }
    public boolean Drive_Score_2() {
        return gamepad1.a && gamepad1.dpad_down;
    }
    public boolean Drive_Preload() {
        return gamepad1.x && gamepad1.dpad_left;
    }
    public boolean Drive_Park() {
        return gamepad1.b && gamepad1.dpad_right;
    }

    public double Intake_Reverse_Sinner() {
        return gamepad2.right_trigger;
    }
    public double Launcher_Reverse_Motor() {
        return gamepad2.left_trigger;
    }

    public boolean Transfer_Eject() {
        return gamepad2.a;
    }
    public boolean Transfer_State_Change() {
        return gamepad2.x;
    }

    public boolean Obelisk_Tag() {
        return gamepad2.share;
    }

    public boolean Terminate_OpMode() {
        return gamepad1.left_bumper && gamepad1.right_bumper;
    }
    public boolean Start_OpMode() {
        return gamepad1.options;
    }

    public void Rumble_Gamepad_1(int milliseconds) {
        gamepad1.rumble(milliseconds);
    }
    public void Rumble_Gamepad_2(int milliseconds) {
        gamepad2.rumble(milliseconds);
    }
    public void Rumble_Both(int milliseconds) {
        gamepad1.rumble(milliseconds);
        gamepad2.rumble(milliseconds);
    }
}
