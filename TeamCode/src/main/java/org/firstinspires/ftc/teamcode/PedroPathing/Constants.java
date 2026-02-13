package org.firstinspires.ftc.teamcode.PedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.8841)

            .forwardZeroPowerAcceleration(-33.98351283)
            .lateralZeroPowerAcceleration(-71.36898706)

            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)

            .translationalPIDFCoefficients(new PIDFCoefficients(0.06, 0, 0.005, 0.1))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.005, 0.01))

            .headingPIDFCoefficients(new PIDFCoefficients(0.7, 0, 0.01, 0))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(2.5, 0, 0.095, 0))

            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.009, 0, 0.0001, 0.6, 0.1))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.02, 0, 0.000015, 0.6, 0.0))

            .centripetalScaling(0.0008);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRightMotor")
            .rightRearMotorName("backRightMotor")
            .leftRearMotorName("backLeftMotor")
            .leftFrontMotorName("frontLeftMotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)


            .xVelocity(64.72621551)
            .yVelocity(67.9199361);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(5.12205)
            .strafePodX(2.49441)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.995, 100, 0.3, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
