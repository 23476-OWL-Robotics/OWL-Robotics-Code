package org.firstinspires.ftc.teamcode.TeleOp.FieldCentric;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class FCdrivecontrol extends FieldCentric {

    @Override
    public void runOpMode() {

    }

    public IMU imu_IMU;
    public DcMotor backLeftMotor;
    public DcMotor frontLeftMotor;
    public DcMotor backRightMotor;
    public DcMotor frontRightMotor;

    YawPitchRollAngles myYawPitchRollAngles;
    public double front_left_power;
    public float X_stick;
    public double BotHeading;
    public double front_right_power;
    public double back_left_power;
    public double back_right_power;
    public float Y_stick;
    public double rotX;
    public double Turning;
    public int heading_divisoin;
    public double rotY;

    /**
     * this function rotates an xy axis set by the imu BotHeading. this
     * function should turn any onmidrive into a field centric drive
     */
    public void rotate_X_and_Y() {
        myYawPitchRollAngles = imu_IMU.getRobotYawPitchRollAngles();
        BotHeading = myYawPitchRollAngles.getYaw(AngleUnit.DEGREES) / heading_divisoin;
        rotX = X_stick * Math.cos(-BotHeading / 180 * Math.PI) - Y_stick * Math.sin(-BotHeading / 180 * Math.PI);
        rotY = X_stick * Math.sin(-BotHeading / 180 * Math.PI) + Y_stick * Math.cos(-BotHeading / 180 * Math.PI);
    }

    /**
     * Describe this function...
     */
    public void drive_control() {
        front_left_power = rotY + rotX + Turning;
        front_right_power = (rotY - rotX) - Turning;
        back_left_power = (rotY - rotX) + Turning;
        back_right_power = (rotY + rotX) - Turning;
    }

    /**
     * this initializes the imu
     */
    public void set_up_imu() {
        imu_IMU.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu_IMU.resetYaw();
    }


    /**
     * Describe this function...
     */
    public void telemetry2() {
        telemetry.addData("x input", X_stick);
        telemetry.addData("y input", Y_stick);
        telemetry.addData("my yaw pitch roll", myYawPitchRollAngles);
        telemetry.addData("BotHeading var", BotHeading);
        telemetry.addData("turning", Turning);
        telemetry.addData("rotY", rotY);
        telemetry.addData("rotX", rotX);
        telemetry.update();
    }

    /**
     * Describe this function...
     */
    public void motor_setup() {
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Describe this function...
     */
    public void motor_power_sets() {
        backLeftMotor.setPower(back_left_power);
        backRightMotor.setPower(back_right_power);
        frontLeftMotor.setPower(front_left_power);
        frontRightMotor.setPower(front_right_power);
    }
}
