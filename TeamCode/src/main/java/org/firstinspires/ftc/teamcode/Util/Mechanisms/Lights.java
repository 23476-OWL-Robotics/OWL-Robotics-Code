package org.firstinspires.ftc.teamcode.Util.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util.Utilities;

public class Lights {

    OpMode opMode;
    HardwareMap hardwareMap;
    Telemetry telemetry;

    Servo leftLED;
    Servo rightLED;

    final double Color_Red = 0.305;
    final double Color_Green = 0.5;
    final double Color_Blue = 0.611;
    final double Color_Purple = 0.7;
    final double Off = 0.0;

    public Lights(OpMode o) {
        this.opMode = o;
        this.hardwareMap = o.hardwareMap;
        this.telemetry = o.telemetry;
    }

    public void init() {
        leftLED = hardwareMap.get(Servo.class, "leftLED");
        rightLED = hardwareMap.get(Servo.class, "rightLED");

        Utilities.Set_PWM_Range(leftLED, new PwmControl.PwmRange(500, 2500));
        Utilities.Set_PWM_Range(rightLED, new PwmControl.PwmRange(500, 2500));

    }

    public void Set_Red() {
        Set_Color(Color_Red);
    }
    public void Set_Green() {
        Set_Color(Color_Green);
    }
    public void Set_Blue() {
        Set_Color(Color_Blue);
    }
    public void Set_Purple() {
        Set_Color(Color_Purple);
    }
    public void Set_Off(){
        Set_Color(Off);
    }

    private void Set_Color(double color) {
        leftLED.setPosition(color);
        rightLED.setPosition(color);
    }
}
