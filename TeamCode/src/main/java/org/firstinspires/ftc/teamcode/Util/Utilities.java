package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.opencv.core.Point3;

public class Utilities {

    public static final Point3 BlueGoalPose = new Point3(2, 140, 38.75);
    public static final Point3 RedGoalPose = new Point3(142, 140, 38.75);

    public static ArtifactPattern PatternID_21 = new ArtifactPattern(ArtifactType.GREEN, ArtifactType.PURPLE, ArtifactType.PURPLE);
    public static ArtifactPattern PatternID_22 = new ArtifactPattern(ArtifactType.PURPLE, ArtifactType.GREEN, ArtifactType.PURPLE);
    public static ArtifactPattern PatternID_23 = new ArtifactPattern(ArtifactType.PURPLE, ArtifactType.PURPLE, ArtifactType.GREEN);

    public static final double BlueTeleHeadingOffset = 180;
    public static final double RedTeleHeadingOffset = 0;

    public static void Set_PWM_Range(Servo servo, PwmControl.PwmRange range) {
        ServoControllerEx controllerEx = (ServoControllerEx) servo.getController();

        controllerEx.setServoPwmRange(servo.getPortNumber(), range);
    }
}
