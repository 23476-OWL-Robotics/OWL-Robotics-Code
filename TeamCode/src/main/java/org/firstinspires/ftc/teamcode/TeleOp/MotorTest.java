package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Util.Timer;

@Disabled
@TeleOp
public class MotorTest extends OpMode {

    enum SelectedMotor {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT,
    }

    DcMotorEx frontLeftMotor;
    DcMotorEx frontRightMotor;
    DcMotorEx backLeftMotor;
    DcMotorEx backRightMotor;

    SelectedMotor state;

    Timer buttonTimer;

    @Override
    public void init() {
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        
        state = SelectedMotor.BACK_LEFT;

        buttonTimer = new Timer();
    }

    @Override
    public void loop() {

        buttonTimer.loop();

        if (gamepad1.dpad_up && buttonTimer.isFinished()) {
            switch (state) {
                case FRONT_LEFT: state = SelectedMotor.FRONT_RIGHT; buttonTimer.setMillisecondTimer(500); break;
                case BACK_LEFT: state = SelectedMotor.FRONT_LEFT; buttonTimer.setMillisecondTimer(500); break;
                case FRONT_RIGHT: state = SelectedMotor.BACK_RIGHT; buttonTimer.setMillisecondTimer(500); break;
                case BACK_RIGHT: state = SelectedMotor.BACK_LEFT; buttonTimer.setMillisecondTimer(500); break;
            }
        }

        switch (state) {
            case FRONT_LEFT: {
                if (gamepad1.a) {
                    frontLeftMotor.setPower(1);
                } else {
                    frontLeftMotor.setPower(0);
                }
                break;
            }
            case BACK_LEFT: {
                if (gamepad1.a) {
                    backLeftMotor.setPower(1);
                } else {
                    backLeftMotor.setPower(0);
                }
                break;
            }
            case FRONT_RIGHT: {
                if (gamepad1.a) {
                    frontRightMotor.setPower(1);
                } else {
                    frontRightMotor.setPower(0);
                }
                break;
            }
            case BACK_RIGHT: {
                if (gamepad1.a) {
                    backRightMotor.setPower(1);
                } else {
                    backRightMotor.setPower(0);
                }
                break;
            }
        }

        telemetry.addData("Motor", state);
        telemetry.addData("Timer", buttonTimer.getMillisecondsRemaining());
        telemetry.update();
    }
}
