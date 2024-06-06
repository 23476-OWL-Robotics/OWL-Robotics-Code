package org.firstinspires.ftc.teamcode.RoadRunner.Tests.TeleOp;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.math.BigInteger;

@TeleOp
public class MainDrive extends LinearOpMode{

    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;

    public Encoder par0;
    public Encoder par1;
    public Encoder perp;

    public void runOpMode() {

        //ToDo: Set Robot Motor Names
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        //ToDo: Set Encoder Names to Matching Motor Port Names
        par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "frontRightMotor")));
        par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "frontLeftMotor")));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "backRightMotor")));

        //ToDo: Set Motor Directions
        //  frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //ToDo: Set Encoder Directions
        //  par0.setDirection(DcMotorSimple.Direction.REVERSE);
        par1.setDirection(DcMotorSimple.Direction.REVERSE);
        perp.setDirection(DcMotorSimple.Direction.REVERSE);

        //Create GamePad JoyStick doubles
        double y;
        double x;

        //Create GamePad Trigger doubles
        double l;
        double r;

        //Create Encoder int
        int par0Position;
        int par1Position;
        int perpPosition;

        waitForStart();
        if (opModeIsActive()) {

            //Set Motors to Break
            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            while (opModeIsActive()) {

                //Set doubles to GamePad Values
                y = gamepad1.left_stick_y;
                x = gamepad1.right_stick_x;
                l = gamepad1.left_trigger;
                r = gamepad1.right_trigger;

                //Set Encoder Values
                par0Position = par0.getPositionAndVelocity().position;
                par1Position = par1.getPositionAndVelocity().position;
                perpPosition = perp.getPositionAndVelocity().position;

                //Drive Code
                if (y > 0.2 || y < -0.2 || x > 0.2 || x < -0.2) {
                    frontLeftMotor.setPower(y - x);
                    frontRightMotor.setPower(y + x);
                    backLeftMotor.setPower(y - x);
                    backRightMotor.setPower(y + x);
                } else if (l > 0.2 || l < -0.2 || r > 0.2 || r < -0.2) {
                    frontLeftMotor.setPower(l - r);
                    frontRightMotor.setPower(-l + r);
                    backLeftMotor.setPower(-l + r);
                    backRightMotor.setPower(l - r);
                } else {
                    frontLeftMotor.setPower(0);
                    frontRightMotor.setPower(0);
                    backLeftMotor.setPower(0);
                    backRightMotor.setPower(0);
                }

                telemetry.addData("Par0 = ", par0Position);
                telemetry.addData("Par1 = ", par1Position);
                telemetry.addData("Perp = ", perpPosition);
                telemetry.update();
            }
        }
    }
}
