package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Utilities.PIDF_Controller.ControllerParams.ArmControllerParams;
import org.firstinspires.ftc.teamcode.Utilities.PIDF_Controller.PIDF_Controller;

@Config
@Autonomous
public class ControllerLoopTest extends LinearOpMode {

    public static class Rotate {
        DcMotorEx motor;

        public Rotate(HardwareMap hardwareMap) {
            motor = hardwareMap.get(DcMotorEx.class, "motor");
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        ArmControllerParams controllerParams = new ArmControllerParams();
        PIDF_Controller controller = new PIDF_Controller(controllerParams.params, motor);

        public class RotateLeft implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket p) {

                if (!initialized) {
                    controller.rotateTo(1);
                    initialized = true;
                }

                double pos = motor.getCurrentPosition();
                p.put("Position: ", pos);
                if (controller.targetReached) {
                    controller.stopController();
                    return true;
                } else {
                    return false;
                }
            }
        }
        public class RotateRight implements Action {

            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket p) {

                if (!initialized) {
                    controller.rotateTo(0);
                    initialized = true;
                }

                double pos = motor.getCurrentPosition();
                p.put("Position: ", pos);
                if (controller.targetReached) {
                    controller.stopController();
                    return true;
                } else {
                    return false;
                }
            }
        }
        public Action rotateLeft() {
            return new RotateLeft();
        }
        public Action rotateRight() {
            return new RotateRight();
        }
    }

    @Override
    public void runOpMode() {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Rotate rotate = new Rotate(hardwareMap);

        waitForStart();
        if (opModeIsActive()) {
            Actions.runBlocking(
                    new SequentialAction(
                            rotate.rotateLeft(),
                            rotate.rotateRight()
                    )
            );
        }
    }
}
