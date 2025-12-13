package org.firstinspires.ftc.teamcode.Util.Mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Util.GamepadMappings;

public class DriveTrain {

    OpMode opMode;
    Telemetry telemetry;

    HardwareMap hardwareMap;

    DcMotorEx frontLeftMotor;
    DcMotorEx frontRightMotor;
    DcMotorEx backLeftMotor;
    DcMotorEx backRightMotor;

    IMU imu;

    double heading;
    double x, y, turn;
    double rotX, rotY;
    double slow;

    double imuOffset;

    double backLeftPower, backRightPower, frontLeftPower, frontRightPower;

    public DriveTrain(OpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
    }

    public void init(double imuOffset) {
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

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        ));
        imu.resetYaw();

        this.imuOffset = imuOffset;
    }

    public void FieldCentric(GamepadMappings m) {

        // Get the heading from the IMU
        heading = -(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + imuOffset);

        // x, y, and turn values from the GamepadMappings
        x = m.Drive_Stick_X();
        y = -m.Drive_Stick_Y();
        turn = -m.Drive_Stick_Turn();

        // rotateX and rotateY are found from trigonometric functions
        rotX = x * Math.cos(heading / 180 * Math.PI) - y * Math.sin(heading / 180 * Math.PI);
        rotY = x * Math.sin(heading / 180 * Math.PI) + y * Math.cos(heading / 180 * Math.PI);

        // Set the motor powers to a combination of rotX, rotY, and turn;
        frontLeftPower = (rotY + rotX) - turn;
        frontRightPower = (rotY - rotX) + turn;
        backLeftPower = (rotY - rotX) - turn;
        backRightPower = (rotY + rotX) + turn;

        // If the driver wants to slow the robot down
        slow = Math.max(0.30, 1 - m.Drive_Slow_Trigger());

        // Set the Motor Powers
        backLeftMotor.setPower(backLeftPower * slow);
        backRightMotor.setPower(backRightPower * slow);
        frontLeftMotor.setPower(frontLeftPower * slow);
        frontRightMotor.setPower(frontRightPower * slow);
    }

    public void RobotCentric(GamepadMappings m) {

        heading = -(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + imuOffset);

        y = -m.Drive_Stick_Y();
        turn = -m.Drive_Stick_Turn();

        frontLeftPower = y - turn;
        frontRightPower = y + turn;
        backLeftPower = y - turn;
        backRightPower = y + turn;

        slow = Math.max(0.10, 1 - m.Drive_Slow_Trigger());

        backLeftMotor.setPower(backLeftPower * slow);
        backRightMotor.setPower(backRightPower * slow);
        frontLeftMotor.setPower(frontLeftPower * slow);
        frontRightMotor.setPower(frontRightPower * slow);
    }

    public void Telemetry() {
        telemetry.addLine("----Motor Powers----");
        telemetry.addData("Front Left Power", frontLeftPower);
        telemetry.addData("Front Right Power", frontRightPower);
        telemetry.addData("Back Left Power", backLeftPower);
        telemetry.addData("Back Right Power", backRightPower);
        telemetry.addLine("----Heading----");
        telemetry.addData("Heading", heading);
    }

    public double getHeading() {
        return heading;
    }

    public void resetHeading() {
        imuOffset = 0;
        imu.resetYaw();
    }
}
