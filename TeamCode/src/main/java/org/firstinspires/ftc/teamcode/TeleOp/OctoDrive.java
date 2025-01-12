package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Drive")
@Disabled
public class OctoDrive extends LinearOpMode {

    DcMotor north;
    DcMotor east;
    DcMotor south;
    DcMotor west;

    IMU imu_IMU;

    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override
    public void runOpMode() {
        double heading;
        int heading_divisoin;
        YawPitchRollAngles myYawPitchRollAngles;
        float X_move;
        float Y_move;
        double Turning;
        double rotX;
        double rotY;
        double south_power;
        double north_power;
        double west_power;
        double east_power;
        double speedModifier = 0.5;

        north = hardwareMap.get(DcMotor.class, "north");
        east = hardwareMap.get(DcMotor.class, "east");
        imu_IMU = hardwareMap.get(IMU.class, "imu");
        south = hardwareMap.get(DcMotor.class, "south");
        west = hardwareMap.get(DcMotor.class, "west");

        north.setDirection(DcMotor.Direction.REVERSE);
        east.setDirection(DcMotor.Direction.REVERSE);
        // Initialize the IMU with non-default settings. To use this block,
        // plug one of the "new IMU.Parameters" blocks into the parameters socket.
        // Create a Parameters object for use with an IMU in a REV Robotics Control Hub or
        // Expansion Hub, specifying the hub's orientation on the robot via the direction that
        // the REV Robotics logo is facing and the direction that the USB ports are facing.
        imu_IMU.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                        )
                )
        );
        waitForStart();
        imu_IMU.resetYaw();
        heading_divisoin = 1;
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                myYawPitchRollAngles = imu_IMU.getRobotYawPitchRollAngles();
                heading = myYawPitchRollAngles.getYaw(AngleUnit.DEGREES) / heading_divisoin;
                X_move = gamepad1.left_stick_x;
                Y_move = -gamepad1.left_stick_y;
                Turning = gamepad1.right_stick_x;
                rotX = X_move * Math.cos(-heading / 180 * Math.PI) - Y_move * Math.sin(-heading / 180 * Math.PI);
                rotY = X_move * Math.sin(-heading / 180 * Math.PI) + Y_move * Math.cos(-heading / 180 * Math.PI);
                south_power = rotX - Turning;
                north_power = rotX + Turning;
                west_power = rotY - Turning;
                east_power = rotY + Turning;
                north.setPower((north_power) * speedModifier);
                south.setPower((south_power) * speedModifier);
                east.setPower((east_power) * speedModifier);
                west.setPower((west_power) * speedModifier);

                if (gamepad2.dpad_up ) {
                    speedModifier += 0.1;
                    try {
                        TimeUnit.MILLISECONDS.sleep(200);
                    } catch (InterruptedException e) {
                        // Nothing
                    }
                } else if (gamepad2.dpad_down) {
                    speedModifier -= 0.1;
                    try {
                        TimeUnit.MILLISECONDS.sleep(200);
                    } catch (InterruptedException e) {
                        // Nothing
                    }
                }
                
                telemetry.addData("Speed", speedModifier);
                telemetry.update();
            }
        }
    }
}