package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.teamcode.Util.Utilities;

@Disabled
@Configurable
@TeleOp(name = "ServoTuner", group = "Tuning")
public class ServoTuning extends OpMode {

    /*
        Launcher Rotation Zero: 0.5
        Launcher Rotation Direction: FORWARD

        Launcher Angle Zero: 0.09
        Left Launcher Angle Servo Direction: FORWARD
        Right Launcher Angle Servo Direction: REVERSE

        Transfer Rotation Zero: 0.0
        Transfer Rotation Slot 1: 0.0
        Transfer Rotation Slot 2: 0.4
        Transfer Rotation Slot 3: 0.8
        Transfer Rotation Direction: REVERSE

        Transfer Lift Zero: 0.1
        Transfer Lift Down: 0.1
        Transfer Lift Up: 0.345
        Left Transfer Lift Direction: FORWARD
        Right Transfer Lift Direction: REVERSE

        LED Colors

     */

    public static double Launcher_Rotation_Position = 0.5;
    public static double Launcher_Angle_Position = 0.07;
    public static double Transfer_Rotation_Position = 0.0;

    public static double Left_LED_Color = 0.0;
    public static double Right_LED_Color = 0.0;

    public static Servo.Direction Launcher_Rotation_Direction = Servo.Direction.FORWARD;
    public static Servo.Direction Launcher_Left_Angle_Direction = Servo.Direction.FORWARD;
    public static Servo.Direction Launcher_Right_Angle_Direction = Servo.Direction.REVERSE;
    public static Servo.Direction Transfer_Rotation_Direction = Servo.Direction.REVERSE;

    Servo launcherLeftRotationServo;
    Servo launcherRightRotationServo;

    Servo launcherLeftAngleServo;
    Servo launcherRightAngleServo;

    Servo transferRotationServo;

    Servo leftLED;
    Servo rightLED;

    TelemetryManager telemetryM;

    @Override
    public void init() {

        launcherLeftRotationServo = hardwareMap.get(Servo.class, "launcherLeftRotationServo");
        launcherRightRotationServo = hardwareMap.get(Servo.class, "launcherRightRotationServo");

        launcherLeftAngleServo = hardwareMap.get(Servo.class, "launcherLeftAngleServo");
        launcherRightAngleServo = hardwareMap.get(Servo.class, "launcherRightAngleServo");

        transferRotationServo = hardwareMap.get(Servo.class, "transferRotationServo");

        leftLED = hardwareMap.get(Servo.class, "leftLED");
        rightLED = hardwareMap.get(Servo.class, "rightLED");

        Utilities.Set_PWM_Range(launcherLeftRotationServo, new PwmControl.PwmRange(500, 2500));
        Utilities.Set_PWM_Range(launcherRightRotationServo, new PwmControl.PwmRange(500, 2500));

        Utilities.Set_PWM_Range(launcherLeftAngleServo, new PwmControl.PwmRange(500, 2500));
        Utilities.Set_PWM_Range(launcherRightAngleServo, new PwmControl.PwmRange(500, 2500));

        Utilities.Set_PWM_Range(transferRotationServo, new PwmControl.PwmRange(500, 2500));

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void loop() {

        transferRotationServo.setDirection(Transfer_Rotation_Direction);

        launcherLeftRotationServo.setDirection(Launcher_Rotation_Direction);
        launcherRightRotationServo.setDirection(Launcher_Rotation_Direction);

        launcherLeftAngleServo.setDirection(Launcher_Left_Angle_Direction);
        launcherRightAngleServo.setDirection(Launcher_Right_Angle_Direction);


        transferRotationServo.setPosition(Transfer_Rotation_Position);

        launcherLeftRotationServo.setPosition(Launcher_Rotation_Position);
        launcherRightRotationServo.setPosition(Launcher_Rotation_Position);

        launcherLeftAngleServo.setPosition(Launcher_Angle_Position);
        launcherRightAngleServo.setPosition(Launcher_Angle_Position);

        leftLED.setPosition(Left_LED_Color);
        rightLED.setPosition(Right_LED_Color);
    }
}
