package org.firstinspires.ftc.teamcode.Utilities.TeleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Utilities.PIDF_Controller.ControllerParams.ArmControllerParams;
import org.firstinspires.ftc.teamcode.Utilities.PIDF_Controller.PIDF_Controller;

import java.util.concurrent.TimeUnit;

public class FieldCentricUtil extends LinearOpMode {

    @Override
    public void runOpMode() {

    }

    private CRServo right;
    private DcMotor backLeftMotor;
    private DcMotor frontLeftMotor;
    private DcMotor leftAssentMotor;
    private DcMotorEx armMotor;
    private DcMotor backRightMotor;
    private DcMotor frontRightMotor;
    private DcMotor intakeMotor;
    private DcMotor rightAssentMotor;
    private IMU imu_IMU;
    private ColorSensor blockDet_REV_ColorRangeSensor;
    private Servo intakePivot;
    private Servo sampleServo;
    private Servo specimenClaw;
    private CRServo left;

    double front_left_power;
    int hang_slide_power;
    double output_trough_position;
    double intake_slide_power;
    double back_left_power;
    double lifter_slide_power;
    double intake_wheel_power;
    double specimen_claw_position;
    double intake_wrist_position;
    double front_right_power;
    double back_right_power;
    double rotX;
    double rotY;
    double speedModifier;
    boolean isUp;
    boolean readjust;

    public void hardwareMaps() {
        right = hardwareMap.get(CRServo.class, "right");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        leftAssentMotor = hardwareMap.get(DcMotor.class, "leftAssentMotor");
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        rightAssentMotor = hardwareMap.get(DcMotor.class, "rightAssentMotor");
        imu_IMU = hardwareMap.get(IMU.class, "imu");
        blockDet_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "blockDet");
        intakePivot = hardwareMap.get(Servo.class, "intakePivot");
        sampleServo = hardwareMap.get(Servo.class, "sampleServo");
        specimenClaw = hardwareMap.get(Servo.class, "specimenClaw");
        left = hardwareMap.get(CRServo.class, "left");
    }

    public void initialize_motors() {
        right.setDirection(CRServo.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        leftAssentMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftAssentMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightAssentMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftAssentMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightAssentMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake_wrist_position = 0.5;
        output_trough_position = 0.485;
    }

    public void initializeServos() {
        intake_wrist_position = 0.5;
        output_trough_position = 0.485;
        specimen_claw_position = 0.73;
        isUp = true;
    }

        public void assent_control(boolean ascend_button, boolean descend_button) {
        if (ascend_button) {
            hang_slide_power = 1;
        } else if (descend_button) {
            hang_slide_power = -1;
        } else {
            hang_slide_power = 0;
        }
    }

    public void motor_power_sets(boolean slowMode) {
        drive_power_sets(slowMode);
        slide_power_sets();
        servo_motor_sets();
    }

    public void output_control(boolean output_up, boolean output_down) {
        if (output_up) {
            output_trough_position = 0.25;
        } else if (output_down) {
            output_trough_position = 0.485;
        }
    }

    ArmControllerParams armControllerParams = new ArmControllerParams();
    PIDF_Controller armController;
    boolean init = false;
    double armPosition = 0;

    int togglePosition;
    boolean override = false;

    public void slide_control(float intake_slide_stick, float lifter_slide_stick,
                               // TODO: Enter the type for argument named deadzone
                               double deadzone) {
        if (!init) {
            armController = new PIDF_Controller(armControllerParams.params, armMotor);
            armController.setMaxSpeed(1);
            armController.setStopOnTargetReached(false);
            init = true;
        }

        if (gamepad2.dpad_up && togglePosition <= 3) {
            togglePosition += 1;
            override = true;
        } else if (gamepad2.dpad_down && togglePosition >= 1) {
            togglePosition -= 1;
            override = true;
        }

        if (lifter_slide_stick > deadzone) {
            armPosition = 37;
            override = false;
        } else if (lifter_slide_stick < -deadzone) {
            armPosition = 0;
            override = false;
        } else if (override) {
            if (togglePosition == 1) {
                armPosition = 6;
            } else if (togglePosition == 2) {
                armPosition = 20;
            } else if (togglePosition == 3) {
                armPosition = 23;
            }
        }

        // Intake slide power
        if (Math.abs(intake_slide_stick) > deadzone) {
            intake_slide_power = intake_slide_stick;
        } else {
            intake_slide_power = 0;
        }
        armController.extendTo(armPosition);
        armController.loopController();
    }

    public void drive_power_sets(boolean slow_mode) {
        if (slow_mode) {
            speedModifier = 0.3;
        } else speedModifier = 1;
        backLeftMotor.setPower(back_left_power * speedModifier);
        backRightMotor.setPower(back_right_power * speedModifier);
        frontLeftMotor.setPower(front_left_power * speedModifier);
        frontRightMotor.setPower(front_right_power * speedModifier);
    }


    public void slide_power_sets() {
        intakeMotor.setPower(-intake_slide_power);
        leftAssentMotor.setPower(hang_slide_power);
        rightAssentMotor.setPower(hang_slide_power);
    }


    public void rotate_x_and_y(float forward_back_stick, float left_right_stick) {
        YawPitchRollAngles myYawPitchRollAngles;
        double heading;

        myYawPitchRollAngles = imu_IMU.getRobotYawPitchRollAngles();
        heading = myYawPitchRollAngles.getYaw(AngleUnit.DEGREES);
        left_right_stick = -left_right_stick;
        rotX = left_right_stick * Math.cos(-heading / 180 * Math.PI) - forward_back_stick * Math.sin(-heading / 180 * Math.PI);
        rotY = left_right_stick * Math.sin(-heading / 180 * Math.PI) + forward_back_stick * Math.cos(-heading / 180 * Math.PI);
    }

    public void telemetry() {
        telemetry.addLine("-----color distance sensor-----");
        telemetry.addData("distance", ((DistanceSensor) blockDet_REV_ColorRangeSensor).getDistance(DistanceUnit.CM));
        telemetry.addData("red", blockDet_REV_ColorRangeSensor.red());
        telemetry.addData("green", blockDet_REV_ColorRangeSensor.green());
        telemetry.addData("blue", blockDet_REV_ColorRangeSensor.blue());
        telemetry.addData("red", red);
        telemetry.addData("yellow", yellow);
        telemetry.addData("blue", blue);
        telemetry.addLine("-----intake assembly-----");
        telemetry.addData("intake slide position", intakeMotor.getCurrentPosition());
        telemetry.addData("intake slide power", intake_slide_power);
        telemetry.addData("intake wheel power", intake_wheel_power);
        telemetry.addData("intake wrist position", intake_wrist_position);
        telemetry.addData("IsUp", isUp);
        telemetry.addLine("-----lifter assembly-----");
        telemetry.addData("lifter slide position", armMotor.getCurrentPosition());
        telemetry.addData("lifter slide power", lifter_slide_power);
        telemetry.addData("output trough position", output_trough_position);
        telemetry.addData("specimen claw position", specimen_claw_position);
        telemetry.update();
    }

    public void red_intake_control(double intake_wheels_in,
                                    double intake_wheels_out,
                                    boolean intake_pivot_up,
                                    boolean intake_pivot_down,
                                    boolean intake_pivot_reset,
                                    double deadZone) {
        loopSensor();

        if (!isUp && sensorDistance > 3.5 && sensorDistance < 4.5) {
            intake_wheel_power = -0.3;
        } else if (intake_wheels_out > deadZone) {
            intake_wheel_power = 1;
        } else if (intake_wheels_in > deadZone) {
            intake_wheel_power = -1;
        } else if (intake_wrist_position < 0.3) {
            intake_wheel_power = 1;
        }else if (intakePivot.getPosition() > 0.60){
            intake_wheel_power = -1;
            intake_wrist_position = 0.5;
        } else {
            intake_wheel_power = 0;
        }
        if (intake_pivot_up) {
            intake_wrist_position = 0.69;
            isUp = true;
        } else if (intake_pivot_down) {
            intake_wrist_position = 0.12;
            intake_wheel_power = 1;
            isUp = false;
        } else if (intake_pivot_reset) {
            intake_wrist_position = 0.5;
            isUp = true;

        } else if (!isUp && sensorDistance < 1 && red || !isUp && sensorDistance < 1 && yellow) {
            intake_wrist_position = 0.50;
        } else if (!isUp && sensorDistance < 1 && blue) {
            intake_wrist_position = 0.31;
            intake_wheel_power = -1;
        }
    }

    public void blue_intake_control(double intake_wheels_in,
                                   double intake_wheels_out,
                                   boolean intake_pivot_up,
                                   boolean intake_pivot_down,
                                   boolean intake_pivot_reset,
                                   double deadZone) {

        loopSensor();

        if (!isUp && sensorDistance > 3.5 && sensorDistance < 4.5) {
            intake_wheel_power = -0.3;
        } else if (intake_wheels_out > deadZone) {
            intake_wheel_power = 1;
        } else if (intake_wheels_in > deadZone) {
            intake_wheel_power = -1;
        } else if (intake_wrist_position < 0.3) {
            intake_wheel_power = 1;
        }else if (intakePivot.getPosition() > 0.60){
            intake_wheel_power = -1;
            intake_wrist_position = 0.5;
        } else {
            intake_wheel_power = 0;
        }
        if (intake_pivot_up) {
            intake_wrist_position = 0.69;
            isUp  = true;
        } else if (intake_pivot_down) {
            intake_wrist_position = 0.12;
            intake_wheel_power = 1;
            isUp = false;
        } else if (intake_pivot_reset) {
            intake_wrist_position = 0.5;
            isUp = true;
        }else if (!isUp && sensorDistance < 1 && blue || !isUp && sensorDistance < 1 && yellow){
            intake_wrist_position = 0.50;
        } else if (!isUp && sensorDistance < 1 && red){
            intake_wrist_position = 0.31;
            intake_wheel_power = -1;
        }
    }

    public void specimen_control(boolean grab_specimen, boolean release_specimen) {
        if (grab_specimen) {
            specimen_claw_position = 0.73;
        } else if (release_specimen) {
            specimen_claw_position = 1;
        }
    }

    public void servo_motor_sets() {
        intakePivot.setPosition(intake_wrist_position);
        sampleServo.setPosition(output_trough_position);
        specimenClaw.setPosition(specimen_claw_position);
        left.setPower(intake_wheel_power);
        right.setPower(intake_wheel_power);
    }


    public void set_up_imu() {
        // Create a RevHubOrientationOnRobot object for use with an IMU in a REV Robotics Control
        // Hub or Expansion Hub, specifying the hub's orientation on the robot via the direction
        // that the REV Robotics logo is facing and the direction that the USB ports are facing.
        imu_IMU.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu_IMU.resetYaw();
    }

    public void drive_control(float turning_stick) {
        front_left_power = rotY + rotX + turning_stick;
        front_right_power = (rotY - rotX) - turning_stick;
        back_left_power = (rotY - rotX) + turning_stick;
        back_right_power = (rotY + rotX) - turning_stick;
    }

    public boolean red = false;
    public boolean yellow = false;
    public boolean blue = false;

    public double sensorDistance;

    public void loopSensor() {
        sensorDistance = ((DistanceSensor) blockDet_REV_ColorRangeSensor).getDistance(DistanceUnit.CM);

        if (blockDet_REV_ColorRangeSensor.red() > 4000 && blockDet_REV_ColorRangeSensor.green() > 6000) {
            yellow = true;
        } else if (blockDet_REV_ColorRangeSensor.blue() > 3000) {
            blue = true;
        } else if (blockDet_REV_ColorRangeSensor.red() > 3000) {
            red = true;
        } else {
            red = false;
            yellow = false;
            blue = false;
        }
    }
}
