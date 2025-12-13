package org.firstinspires.ftc.teamcode.Util.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util.Timer;
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

    double currentColor = 0.0;

    private boolean blink = false;
    private boolean blinkOff = true;
    private int blinkNumber = 0;
    private Timer blinkTimer;

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

        blinkTimer = new Timer();
    }

    public void loop() {
        blinkTimer.loop();

        if (!blink) {
            return;
        }

        if (!blinkTimer.isFinished()) {
            return;
        }

        if (blinkOff) {
            Set_Color(Off);
            blink = false;
        } else {
            Set_Color(currentColor);
            blink = true;
        }

        if (blinkNumber != 0) {
            blinkNumber--;
            blinkTimer.setMillisecondTimer(1000);
            return;
        }

        Set_Color(currentColor);
        blink = false;
    }

    public boolean DoneBlinking() {
        return !blink;
    }

    public void Set_Red() {
        Set_Color(Color_Red);
        currentColor = Color_Red;
    }
    public void Set_Green() {
        Set_Color(Color_Green);
        currentColor = Color_Green;
    }
    public void Set_Blue() {
        Set_Color(Color_Blue);
        currentColor = Color_Blue;
    }
    public void Set_Purple() {
        Set_Color(Color_Purple);
        currentColor = Color_Purple;
    }
    public void Set_Off(){
        Set_Color(Off);
        currentColor = Off;
    }

    private void Set_Color(double color) {
        leftLED.setPosition(color);
        rightLED.setPosition(color);
    }

    public void Blink_Red(int n) {
        blinkNumber = n;
        blink = true;
        blinkOff = true;
        currentColor = Color_Red;
        Set_Red();
    }
    public void Blink_Green(int n) {
        blinkNumber = n;
        blink = true;
        blinkOff = true;
        currentColor = Color_Green;
        Set_Green();
    }
    public void Blink_Blue(int n) {
        blinkNumber = n;
        blink = true;
        blinkOff = true;
        currentColor = Color_Blue;
        Set_Blue();
    }
    public void Blink_Purple(int n) {
        blinkNumber = n;
        blink = true;
        blinkOff = true;
        currentColor = Color_Purple;
        Set_Purple();
    }
}
