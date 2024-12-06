package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepAutoSpecimen {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity botBlue = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeBlueDark())
                .build();
        RoadRunnerBotEntity botRed = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setColorScheme(new ColorSchemeRedDark())
                .build();

        botBlue.runAction(botBlue.getDrive().actionBuilder(new Pose2d(-12, 60, Math.toRadians(-90)))
                .setTangent(Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-12, 34, Math.toRadians(-90)), Math.toRadians(-90))
                .waitSeconds(1)
                .setTangent(Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-24, 34, Math.toRadians(-90)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-46, 0, Math.toRadians(-90)), Math.toRadians(-145))
                .setTangent(Math.toRadians(90))
                .lineToY(52)
                .setTangent(Math.toRadians(-90))
                .lineToYSplineHeading(24, Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-56, 12, Math.toRadians(-90)), Math.toRadians(180))
                .setTangent(Math.toRadians(90))
                .lineToY(52)
                .setTangent(Math.toRadians(-90))
                .lineToYSplineHeading(24, Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(-62, 12, Math.toRadians(-90)), Math.toRadians(180))
                .setTangent(Math.toRadians(90))
                .lineToY(52)
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-36, 62, Math.toRadians(90)), Math.toRadians(45))
                .waitSeconds(1)
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(-10, 34, Math.toRadians(-90)), Math.toRadians(-45))
                .waitSeconds(1)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-36, 62, Math.toRadians(90)), Math.toRadians(135))
                .waitSeconds(1)
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(-10, 34, Math.toRadians(-90)), Math.toRadians(-45))
                .waitSeconds(1)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-36, 62, Math.toRadians(90)), Math.toRadians(135))
                .waitSeconds(1)
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(-10, 34, Math.toRadians(-90)), Math.toRadians(-45))
                .waitSeconds(1)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-36, 62, Math.toRadians(90)), Math.toRadians(135))
                .build());

        botRed.runAction(botRed.getDrive().actionBuilder(new Pose2d(12, -60, Math.toRadians(90)))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(12, -34, Math.toRadians(90)), Math.toRadians(90))
                .waitSeconds(1)
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(24, -34, Math.toRadians(90)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(46, 0, Math.toRadians(90)), Math.toRadians(35))
                .setTangent(Math.toRadians(-90))
                .lineToY(-52)
                .setTangent(Math.toRadians(90))
                .lineToYSplineHeading(-24, Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(56, -12, Math.toRadians(90)), Math.toRadians(0))
                .setTangent(Math.toRadians(-90))
                .lineToY(-52)
                .setTangent(Math.toRadians(-90))
                .lineToYSplineHeading(-24, Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(62, -12, Math.toRadians(90)), Math.toRadians(0))
                .setTangent(Math.toRadians(-90))
                .lineToY(-52)
                .setTangent(Math.toRadians(-135))
                .splineToLinearHeading(new Pose2d(36, -62, Math.toRadians(-90)), Math.toRadians(-135))
                .waitSeconds(1)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(10, -34, Math.toRadians(90)), Math.toRadians(135))
                .waitSeconds(1)
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(36, -62, Math.toRadians(-90)), Math.toRadians(-45))
                .waitSeconds(1)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(10, -34, Math.toRadians(90)), Math.toRadians(135))
                .waitSeconds(1)
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(36, -62, Math.toRadians(-90)), Math.toRadians(-45))
                .waitSeconds(1)
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(10, -34, Math.toRadians(90)), Math.toRadians(135))
                .waitSeconds(1)
                .setTangent(Math.toRadians(-45))
                .splineToLinearHeading(new Pose2d(36, -62, Math.toRadians(-90)), Math.toRadians(-45))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(botBlue)
                .addEntity(botRed)
                .start();
    }
}