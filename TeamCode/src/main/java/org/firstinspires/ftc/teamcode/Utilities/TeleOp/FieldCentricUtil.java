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
import org.firstinspires.ftc.teamcode.Utilities.PIDF_Controller.ControllerParams.IntakeControllerParams;
import org.firstinspires.ftc.teamcode.Utilities.PIDF_Controller.ControllerParams.LeftAssentControllerParams;
import org.firstinspires.ftc.teamcode.Utilities.PIDF_Controller.ControllerParams.RightAssentControllerParams;
import org.firstinspires.ftc.teamcode.Utilities.PIDF_Controller.PIDF_Controller;

public class FieldCentricUtil extends LinearOpMode {

    @Override
    public void runOpMode() {

    }

    // Create All Motors
    DcMotorEx frontRightMotor;
    DcMotorEx backRightMotor;
    DcMotorEx frontLeftMotor;
    DcMotorEx backLeftMotor;
    DcMotorEx rightAssentMotor;
    DcMotorEx leftAssentMotor;
    DcMotorEx armMotor;
    DcMotorEx intakeMotor;

    // Create All Servos
    Servo intakePivot;
    Servo sampleServo;
    Servo specimenServo;


    // Create All CR Servos
    CRServo left;
    CRServo right;

    // Create sensor and imu
    ColorSensor sensor;
    IMU imu;

    // Drive Motor Powers
    double front_left_power;
    double front_right_power;
    double back_left_power;
    double back_right_power;

    // Slide Motor Powers
    double arm_slide_power;
    double intake_slide_power;

    // Servo Positions
    double sample_servo_position;
    double specimen_servo_position;
    double intake_pivot_position;

    // Servo Powers
    double intake_wheel_power;

    // Hang Motor Powers
    int hang_slide_power;

    // Other Variables
    double rotX;
    double rotY;
    double speedModifier;
    boolean isUp;
    boolean doingIntake;
    boolean touchSet;
    float TFoneRefX;
    float TFoneRefY;
    float TFtwoRefX;
    float TFtwoRefY;



    // Hardware Maps
    public void hardwareMaps() {
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "backRightMotor");
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "backLeftMotor");

        rightAssentMotor = hardwareMap.get(DcMotorEx.class, "rightAssentMotor");
        leftAssentMotor = hardwareMap.get(DcMotorEx.class, "leftAssentMotor");

        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");

        intakePivot = hardwareMap.get(Servo.class, "intakePivot");
        sampleServo = hardwareMap.get(Servo.class, "sampleServo");
        specimenServo = hardwareMap.get(Servo.class, "specimenClaw");

        left = hardwareMap.get(CRServo.class, "left");
        right = hardwareMap.get(CRServo.class, "right");

        sensor = hardwareMap.get(ColorSensor.class, "blockDet");

        imu = hardwareMap.get(IMU.class, "imu");

    }

    // Motor Initialization
    // Sets all Motors to Break
    public void initialize_motors() {
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightAssentMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftAssentMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        leftAssentMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(CRServo.Direction.REVERSE);

        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftAssentMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightAssentMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        touchSet = false;
    }

    // Servo Positions on opModeInInit()
    public void initializeServos() {
        intake_pivot_position = 0.5;
        sample_servo_position = 0.485;
        specimen_servo_position = 0.73;
        isUp = true;
    }

    // IMU Initialization
    public void set_up_imu() {
        // Create a RevHubOrientationOnRobot object for use with an IMU in a REV Robotics Control
        // Hub or Expansion Hub, specifying the hub's orientation on the robot via the direction
        // that the REV Robotics logo is facing and the direction that the USB ports are facing.
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        imu.resetYaw();
    }

    //changes stick intake to field centric
    public void rotate_x_and_y(double forward_back_stick, double left_right_stick) {
        YawPitchRollAngles myYawPitchRollAngles;
        double heading;

        myYawPitchRollAngles = imu.getRobotYawPitchRollAngles();
        heading = myYawPitchRollAngles.getYaw(AngleUnit.DEGREES);
        left_right_stick = -left_right_stick;
        rotX = left_right_stick * Math.cos(-heading / 180 * Math.PI) - forward_back_stick * Math.sin(-heading / 180 * Math.PI);
        rotY = left_right_stick * Math.sin(-heading / 180 * Math.PI) + forward_back_stick * Math.cos(-heading / 180 * Math.PI);
    }

    //provide 360 degree movement from a stick
    public void drive_control(double turning_stick) {
      if(gamepad2.touchpad_finger_1 && gamepad2.touchpad_finger_2){
          if(!touchSet){
              touchSet = true;
              if(gamepad2.touchpad_finger_1_x > gamepad2.touchpad_finger_2_x){
                  TFoneRefX = gamepad2.touchpad_finger_1_x;
                  TFoneRefY = gamepad2.touchpad_finger_1_y;
                  TFtwoRefX = gamepad2.touchpad_finger_2_x;
                  TFtwoRefY = gamepad2.touchpad_finger_2_y;
              }else{
                  TFtwoRefX = gamepad2.touchpad_finger_1_x;
                  TFtwoRefY = gamepad2.touchpad_finger_1_y;
                  TFoneRefX = gamepad2.touchpad_finger_2_x;
                  TFoneRefY = gamepad2.touchpad_finger_2_y;
              }

          }
          else{
              front_left_power = (gamepad2.touchpad_finger_1_y-TFoneRefY)
                      + (gamepad2.touchpad_finger_1_x-TFoneRefX)
                      + (gamepad2.touchpad_finger_2_x-TFtwoRefX);

              front_right_power = (gamepad2.touchpad_finger_1_y-TFoneRefY)
                      - (gamepad2.touchpad_finger_1_x-TFoneRefX)
                      - (gamepad2.touchpad_finger_2_x-TFtwoRefX);

              back_left_power = (gamepad2.touchpad_finger_1_y-TFoneRefY)
                      - (gamepad2.touchpad_finger_1_x-TFoneRefX)
                      + (gamepad2.touchpad_finger_2_x-TFtwoRefX);

              back_right_power = (gamepad2.touchpad_finger_1_y-TFoneRefY)
                      + (gamepad2.touchpad_finger_1_x-TFoneRefX)
                      - (gamepad2.touchpad_finger_2_x-TFtwoRefX);
          }
      }else{
          front_left_power = rotY + rotX + turning_stick;
          front_right_power = (rotY - rotX) - turning_stick;
          back_left_power = (rotY - rotX) + turning_stick;
          back_right_power = (rotY + rotX) - turning_stick;
      }

    }

    //sets motor power sets
    public void power_sets(boolean slowMode) {
        drive_power_sets(slowMode);
        slide_power_sets();
        servo_power_sets();
    }

    //sets wrist servo to positions set by slide control
    public void sample_control(boolean output_up, boolean output_down) {
        if (output_up) {
            sample_servo_position = 0.25;
        } else if (output_down) {
            sample_servo_position = 0.485;
        }
    }

    //opens and closes specimen claw
    public void specimen_control(boolean grab_specimen, boolean release_specimen) {
        if (grab_specimen) {
            specimen_servo_position = 0.73;
        } else if (release_specimen) {
            specimen_servo_position = 1;
        }
    }

    //parameter for assent motors
    LeftAssentControllerParams leftAssentControllerParams = new LeftAssentControllerParams();
    PIDF_Controller leftAssentController;

    RightAssentControllerParams rightAssentControllerParams = new RightAssentControllerParams();
    PIDF_Controller rightAssentController;

    boolean assentInti = false;
    double leftAssentPosition;
    double rightAssentPosition;

    // control the asent motors with pidf
    public void assent_control(boolean ascend_button, boolean descend_button) {
        if (!assentInti) {
            leftAssentController = new PIDF_Controller(leftAssentControllerParams.params, leftAssentMotor);
            leftAssentController.setMaxSpeed(1);
            leftAssentController.setStopOnTargetReached(false);

            rightAssentController = new PIDF_Controller(rightAssentControllerParams.params, rightAssentMotor);
            rightAssentController.setMaxSpeed(1);
            rightAssentController.setStopOnTargetReached(false);

            assentInti = true;
        }

        if (ascend_button) {
            hang_slide_power = 1;
        } else if (descend_button) {
            hang_slide_power = -1;
        } else {
            hang_slide_power = 0;
        }
    }

    //paramaters for scoring slides
    ArmControllerParams armControllerParams = new ArmControllerParams();
    PIDF_Controller armController;

    IntakeControllerParams intakeControllerParams = new IntakeControllerParams();
    PIDF_Controller intakeController;

    boolean slideInit = false;
    double armPosition = 0;
    double intakePosition = 0;

    // controls lhe slides for scoring
    public void slide_control(double intake_slide_stick, double lifter_slide_stick, boolean reset, double deadZone) {
        if (!slideInit) {
            armController = new PIDF_Controller(armControllerParams.params, armMotor);
            armController.setMaxSpeed(1);
            armController.setStopOnTargetReached(false);

            intakeController = new PIDF_Controller(intakeControllerParams.params, intakeMotor);
            intakeController.setMaxSpeed(1);
            intakeController.setStopOnTargetReached(false);

            slideInit = true;
        }

        if (gamepad2.dpad_up) {
            armPosition = 23;
        } else if (gamepad2.dpad_down) {
            armPosition = 20;
        }

        if (lifter_slide_stick > deadZone) {
            armPosition = 37;
        } else if (lifter_slide_stick < -deadZone) {
            armPosition = 0;
        }

        // Intake slide power
        if (Math.abs(intake_slide_stick) > deadZone) {
            intakeMotor.setPower(-intake_slide_stick);
        } else {
            if(doingIntake){
                intakeController.extendTo(1);
                intakeController.loopController();
            }else{
                intakeMotor.setPower(0);
            }
        }
        armController.extendTo(armPosition);
        armController.loopController();
    }

    public void red_intake_control(double intake_wheels_in,
                                   double intake_wheels_out,
                                   boolean intake_pivot_up,
                                   boolean intake_pivot_down,
                                   boolean intake_pivot_reset,
                                   double deadZone) {
        loopSensor();

        //automatic and manual control of intake wheels
        if (!isUp && sensorDistance > 3.5 && sensorDistance < 4.5) {
            intake_wheel_power = -0.3;
        } else if (intake_wheels_out > deadZone) {
            intake_wheel_power = 1;
        } else if (intake_wheels_in > deadZone) {
            intake_wheel_power = -1;
        } else if (intake_pivot_position < 0.3) {
            intake_wheel_power = 1;
        }else if (intakePivot.getPosition() > 0.60){
            intake_wheel_power = -1;
            intake_pivot_position = 0.5;
        } else {
            intake_wheel_power = 0;
        }

        //puts the intake pivot to is positions based on inputs
        if (intake_pivot_up) {
            intake_pivot_position = 0.69;
            isUp = true;
        } else if (intake_pivot_down) {
            intake_pivot_position = 0.12;
            intake_wheel_power = 1;
            isUp = false;
        } else if (intake_pivot_reset) {
            intake_pivot_position = 0.5;
            isUp = true;
        } else if (!isUp && sensorDistance < 1 && red || !isUp && sensorDistance < 1 && yellow) {
            intake_pivot_position = 0.50;
            doingIntake = true;
        } else if (!isUp && sensorDistance < 1 && blue) {
            intake_pivot_position = 0.31;
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
        } else if (intake_pivot_position < 0.3) {
            intake_wheel_power = 1;
        }else if (intakePivot.getPosition() > 0.60){
            intake_wheel_power = -1;
            intake_pivot_position = 0.5;
        } else {
            intake_wheel_power = 0;
        }

        if (intake_pivot_up) {
            intake_pivot_position = 0.69;
            isUp  = true;
        } else if (intake_pivot_down) {
            intake_pivot_position = 0.12;
            intake_wheel_power = 1;
            isUp = false;
        } else if (intake_pivot_reset) {
            intake_pivot_position = 0.5;
            isUp = true;
        } else if (!isUp && sensorDistance < 1 && blue || !isUp && sensorDistance < 1 && yellow){
            intake_pivot_position = 0.50;
            doingIntake = true;
        } else if (!isUp && sensorDistance < 1 && red){
            intake_pivot_position = 0.31;
            intake_wheel_power = -1;
        }
    }

    public void Dointake(){
        if (intakeController.targetReached){
            intake_pivot_position = 0.69;
            isUp  = true;

        }
    }

    // Power Sets
    public void servo_power_sets() {
        intakePivot.setPosition(intake_pivot_position);
        sampleServo.setPosition(sample_servo_position);
        specimenServo.setPosition(specimen_servo_position);
        left.setPower(intake_wheel_power);
        right.setPower(intake_wheel_power);
    }
    public void slide_power_sets() {
        leftAssentMotor.setPower(hang_slide_power);
        rightAssentMotor.setPower(hang_slide_power);
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

    // Telemetry for Debugging
    public void telemetry() {
        telemetry.addLine("-----color distance sensor-----");
        telemetry.addData("distance", ((DistanceSensor) sensor).getDistance(DistanceUnit.CM));
        telemetry.addData("red", sensor.red());
        telemetry.addData("green", sensor.green());
        telemetry.addData("blue", sensor.blue());
        telemetry.addData("red", red);
        telemetry.addData("yellow", yellow);
        telemetry.addData("blue", blue);
        telemetry.addLine("-----intake assembly-----");
        telemetry.addData("intake slide position", intakeMotor.getCurrentPosition());
        telemetry.addData("intake slide power", intakePosition);
        telemetry.addData("intake wheel power", intake_wheel_power);
        telemetry.addData("intake wrist position", intake_pivot_position);
        telemetry.addData("IsUp", isUp);
        telemetry.addLine("-----lifter assembly-----");
        telemetry.addData("lifter slide position", armMotor.getCurrentPosition());
        telemetry.addData("lifter slide power", arm_slide_power);
        telemetry.addData("output trough position", sample_servo_position);
        telemetry.addData("specimen claw position", specimen_servo_position);
        telemetry.update();
    }

    // Sensor Utilities
    public boolean red = false;
    public boolean yellow = false;
    public boolean blue = false;

    public double sensorDistance;

    public void loopSensor() {
        sensorDistance = ((DistanceSensor) sensor).getDistance(DistanceUnit.CM);

        if (sensor.red() > 4000 && sensor.green() > 6000) {
            yellow = true;
        } else if (sensor.blue() > 3000) {
            blue = true;
        } else if (sensor.red() > 3000) {
            red = true;
        } else {
            red = false;
            yellow = false;
            blue = false;
        }
    }
}
