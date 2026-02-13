package org.firstinspires.ftc.teamcode.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Util.Timer;

@Disabled
@Configurable
@TeleOp(name = "Motor Tuning", group = "Tuning")
public class MotorTuning extends OpMode {

    public static double Intake_Motor_Speed = 0.0;
    public static DcMotorSimple.Direction Intake_Motor_Direction = DcMotorSimple.Direction.FORWARD;

    public static double Lift_Motor_Speed = 0.0;
    public static DcMotorSimple.Direction Lift_Motor_Direction = DcMotorSimple.Direction.REVERSE;

    public static double Launcher_Motor_Speed = 0.0;
    public static DcMotorSimple.Direction Left_Launch_Motor_Direction = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction Right_Launch_Motor_Direction = DcMotorSimple.Direction.REVERSE;

    DcMotorEx intakeMotor;
    DcMotorEx liftMotor;
    DcMotorEx leftLaunchMotor;
    DcMotorEx rightLaunchMotor;

    Timer buttonTimer;

    @Override
    public void init() {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        liftMotor = hardwareMap.get(DcMotorEx.class, "transferLiftMotor");
        leftLaunchMotor = hardwareMap.get(DcMotorEx.class, "launcherLeftMotor");
        rightLaunchMotor = hardwareMap.get(DcMotorEx.class, "launcherRightMotor");

        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLaunchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLaunchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftLaunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightLaunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        buttonTimer = new Timer();
    }

    @Override
    public void loop() {
        intakeMotor.setPower(Intake_Motor_Speed);
        intakeMotor.setDirection(Intake_Motor_Direction);

        liftMotor.setPower(Lift_Motor_Speed);
        liftMotor.setDirection(Lift_Motor_Direction);

        leftLaunchMotor.setPower(Launcher_Motor_Speed);
        rightLaunchMotor.setPower(Launcher_Motor_Speed);

        leftLaunchMotor.setDirection(Left_Launch_Motor_Direction);
        rightLaunchMotor.setDirection(Right_Launch_Motor_Direction);

        telemetry.addData("Intake Motor Position", intakeMotor.getCurrentPosition());
        telemetry.addData("Lift Motor Position", liftMotor.getCurrentPosition());
        telemetry.addLine();
        telemetry.addData("Left Launch Motor Velocity", leftLaunchMotor.getVelocity());
        telemetry.addData("Right Launch Motor Velocity", rightLaunchMotor.getVelocity());
        telemetry.update();
    }
}
