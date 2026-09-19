package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

public class Utilities {

    public static void Set_PWM_Range(Servo servo, PwmControl.PwmRange range) {
        ServoControllerEx controllerEx = (ServoControllerEx) servo.getController();
        controllerEx.setServoPwmRange(servo.getPortNumber(), range);
    }
}
