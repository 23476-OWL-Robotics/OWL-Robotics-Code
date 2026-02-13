package org.firstinspires.ftc.teamcode.Util.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;

public class AutoPaths {

    public static class Auto_Blue_Goal {

        public PathChain viewObelisk;
        public PathChain scorePreloads;

        public PathChain lineupPickup1;
        public PathChain lineupPickup2;

        public PathChain grabArtifact1Pickup1;
        public PathChain grabArtifact2Pickup1;
        public PathChain grabArtifact3Pickup1;

        public PathChain grabArtifact1Pickup2;
        public PathChain grabArtifact2Pickup2;
        public PathChain grabArtifact3Pickup2;

        public PathChain scorePickup1;
        public PathChain scorePickup2;

        public PathChain park;
        public PathChain creekPark;

        public Auto_Blue_Goal(Follower follower) {
            CreatePaths(follower);
        }

        public static Pose startPose = new Pose(18, 124.5, Math.toRadians(-129));
        protected static Pose viewObeliskPose = new Pose(54, 125, Math.toRadians(235));

        protected static Pose scorePose1 = new Pose(40, 100, Math.toRadians(-45));

        protected static Pose startPickup1Pose = new Pose(44, 84, Math.toRadians(180));
        protected static Pose startPickup2Pose = new Pose(41, 60, Math.toRadians(180));

        protected static Pose startPickup1Control = new Pose(60, 96);
        protected static Pose startPickup2Control = new Pose(55, 84);

        protected static Pose stop1Pickup1Pose = new Pose(38, 84, Math.toRadians(180));
        protected static Pose stop2Pickup1Pose = new Pose(31.5, 84, Math.toRadians(180));

        protected static Pose stop1Pickup2Pose = new Pose(38, 60, Math.toRadians(180));
        protected static Pose stop2Pickup2Pose = new Pose(31.5, 60, Math.toRadians(180));

        protected static Pose endPickup1Pose = new Pose(24, 84, Math.toRadians(180));
        protected static Pose endPickup2Pose = new Pose(24, 60, Math.toRadians(180));

        protected static Pose scorePickup1Control = new Pose(38, 88);
        protected static Pose scorePickup2Control = new Pose(38, 60);

        protected static Pose endPose = new Pose(25, 70, Math.toRadians(-90));
        protected static Pose creekEndPose = new Pose(56, 136, Math.toRadians(0));

        PathConstraints constraints = new PathConstraints(
                0.995,
                0.1,
                0.1,
                0.007,
                100,
                0.3,
                10,
                1);

        void CreatePaths(Follower follower) {

            follower.setConstraints(constraints);

            viewObelisk = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    startPose,
                                    viewObeliskPose
                            )
                    )
                    .setLinearHeadingInterpolation(startPose.getHeading(), viewObeliskPose.getHeading())
                    .setConstraints(constraints)
                    .build();

            scorePreloads = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    viewObeliskPose,
                                    scorePose1
                            )
                    )
                    .setLinearHeadingInterpolation(viewObeliskPose.getHeading(), scorePose1.getHeading())
                    .setConstraints(constraints)
                    .build();

            lineupPickup1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    scorePose1,
                                    startPickup1Control,
                                    startPickup1Pose
                            )
                    )
                    .setLinearHeadingInterpolation(scorePose1.getHeading(), startPickup1Pose.getHeading())
                    .setConstraints(constraints)
                    .setTranslationalConstraint(0.1)
                    .build();

            grabArtifact1Pickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(startPickup1Pose, stop1Pickup1Pose))
                    .setConstantHeadingInterpolation(startPickup1Pose.getHeading())
                    .setConstraints(constraints)
                    .setTimeoutConstraint(1300)
                    .build();

            grabArtifact2Pickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(stop1Pickup1Pose, stop2Pickup1Pose))
                    .setConstantHeadingInterpolation(startPickup1Pose.getHeading())
                    .setConstraints(constraints)
                    .setTimeoutConstraint(1800)
                    .build();

            grabArtifact3Pickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(stop2Pickup1Pose, endPickup1Pose))
                    .setConstantHeadingInterpolation(startPickup1Pose.getHeading())
                    .setConstraints(constraints)
                    .build();

            scorePickup1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    endPickup1Pose,
                                    scorePickup1Control,
                                    scorePose1
                            )
                    )
                    .setLinearHeadingInterpolation(endPickup1Pose.getHeading(), scorePose1.getHeading())
                    .setConstraints(constraints)
                    .build();


            lineupPickup2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    scorePose1,
                                    startPickup2Control,
                                    startPickup2Pose
                            )
                    )
                    .setLinearHeadingInterpolation(scorePose1.getHeading(), startPickup2Pose.getHeading())
                    .setConstraints(constraints)
                    .build();

            grabArtifact1Pickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(startPickup2Pose, stop1Pickup2Pose))
                    .setConstantHeadingInterpolation(startPickup2Pose.getHeading())
                    .setConstraints(constraints)
                    .setTimeoutConstraint(1500)
                    .build();

            grabArtifact2Pickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(stop1Pickup2Pose, stop2Pickup2Pose))
                    .setConstantHeadingInterpolation(startPickup2Pose.getHeading())
                    .setConstraints(constraints)
                    .setTimeoutConstraint(2000)
                    .build();

            grabArtifact3Pickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(stop2Pickup2Pose, endPickup2Pose))
                    .setConstantHeadingInterpolation(startPickup2Pose.getHeading())
                    .setConstraints(constraints)
                    .build();

            scorePickup2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    endPickup2Pose,
                                    scorePickup2Control,
                                    scorePose1
                            )
                    )
                    .setLinearHeadingInterpolation(endPickup2Pose.getHeading(), scorePose1.getHeading())
                    .setConstraints(constraints)
                    .build();

            park = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    scorePose1,
                                    endPose
                            )
                    )
                    .setLinearHeadingInterpolation(scorePose1.getHeading(), endPose.getHeading())
                    .setConstraints(constraints)
                    .build();

            creekPark = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    scorePose1,
                                    creekEndPose
                            )
                    )
                    .setLinearHeadingInterpolation(scorePose1.getHeading(), creekEndPose.getHeading())
                    .setConstraints(constraints)
                    .build();
        }
    }

    public static class Auto_Blue_Wall {

        public PathChain lineupPickup1;

        public PathChain grabArtifact1Pickup1;
        public PathChain grabArtifact2Pickup1;
        public PathChain grabArtifact3Pickup1;

        public PathChain scorePickup1;
        public PathChain park;

        public Auto_Blue_Wall(Follower follower) {
            CreatePaths(follower);
        }

        public static Pose startPose = new Pose(56, 8, Math.toRadians(-90));

        protected static Pose startPickupPose1 = new Pose(44, 34.5, Math.toRadians(180));

        protected static Pose stop1Pickup1Pose = new Pose(38, 34.5, Math.toRadians(180));
        protected static Pose stop2Pickup1Pose = new Pose(31.5, 34.5, Math.toRadians(180));

        protected static Pose endPickup1Pose = new Pose(24, 34.5, Math.toRadians(180));
        protected static Pose scorePickup1Pose = new Pose(56, 12, Math.toRadians(-90));
        protected static Pose endPose = new Pose(34, 12, Math.toRadians(-90));

        PathConstraints constraints = new PathConstraints(
                0.995,
                0.1,
                0.1,
                0.007,
                100,
                0.3,
                10,
                1);

        void CreatePaths(Follower follower) {

            follower.setConstraints(constraints);

            lineupPickup1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    startPose,
                                    startPickupPose1
                            )
                    )
                    .setLinearHeadingInterpolation(startPose.getHeading(), startPickupPose1.getHeading())
                    .setConstraints(constraints)
                    .build();

            grabArtifact1Pickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(startPickupPose1, stop1Pickup1Pose))
                    .setConstantHeadingInterpolation(startPickupPose1.getHeading())
                    .setConstraints(constraints)
                    .setTimeoutConstraint(1300)
                    .build();

            grabArtifact2Pickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(stop1Pickup1Pose, stop2Pickup1Pose))
                    .setConstantHeadingInterpolation(startPickupPose1.getHeading())
                    .setConstraints(constraints)
                    .setTimeoutConstraint(1800)
                    .build();

            grabArtifact3Pickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(stop2Pickup1Pose, endPickup1Pose))
                    .setConstantHeadingInterpolation(startPickupPose1.getHeading())
                    .setConstraints(constraints)
                    .build();

            scorePickup1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    endPickup1Pose,
                                    scorePickup1Pose
                            )
                    )
                    .setLinearHeadingInterpolation(endPickup1Pose.getHeading(), scorePickup1Pose.getHeading())
                    .setConstraints(constraints)
                    .build();

            park = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    scorePickup1Pose,
                                    endPose
                            )
                    )
                    .setLinearHeadingInterpolation(scorePickup1Pose.getHeading(), endPose.getHeading())
                    .setConstraints(constraints)
                    .build();

        }
    }

    public static class Auto_Red_Goal {

        public PathChain viewObelisk;
        public PathChain scorePreloads;

        public PathChain lineupPickup1;
        public PathChain lineupPickup2;

        public PathChain grabArtifact1Pickup1;
        public PathChain grabArtifact2Pickup1;
        public PathChain grabArtifact3Pickup1;

        public PathChain grabArtifact1Pickup2;
        public PathChain grabArtifact2Pickup2;
        public PathChain grabArtifact3Pickup2;

        public PathChain scorePickup1;
        public PathChain scorePickup2;

        public PathChain park;
        public PathChain creekPark;

        public Auto_Red_Goal(Follower follower) {
            CreatePaths(follower);
        }

        public static Pose startPose = Auto_Blue_Goal.startPose.mirror();
        protected static Pose viewObeliskPose = Auto_Blue_Goal.viewObeliskPose.mirror();

        protected static Pose scorePose1 = Auto_Blue_Goal.scorePose1.mirror();

        protected static Pose startPickup1Pose = Auto_Blue_Goal.startPickup1Pose.mirror();

        protected static Pose startPickup2Pose = Auto_Blue_Goal.startPickup2Pose.mirror();

        protected static Pose startPickup1Control = Auto_Blue_Goal.startPickup1Control.mirror();
        protected static Pose startPickup2Control = Auto_Blue_Goal.startPickup2Control.mirror();

        protected static Pose stop1Pickup1Pose = Auto_Blue_Goal.stop1Pickup1Pose.mirror();
        protected static Pose stop2Pickup1Pose = Auto_Blue_Goal.stop2Pickup1Pose.mirror();

        protected static Pose stop1Pickup2Pose = Auto_Blue_Goal.stop1Pickup2Pose.mirror();
        protected static Pose stop2Pickup2Pose = Auto_Blue_Goal.stop2Pickup2Pose.mirror();

        protected static Pose endPickup1Pose = Auto_Blue_Goal.endPickup1Pose.mirror();
        protected static Pose endPickup2Pose = Auto_Blue_Goal.endPickup2Pose.mirror();

        protected static Pose scorePickup1Control = Auto_Blue_Goal.scorePickup1Control.mirror();
        protected static Pose scorePickup2Control = Auto_Blue_Goal.scorePickup2Control.mirror();

        protected static Pose endPose = Auto_Blue_Goal.endPose.mirror();
        protected static Pose creekEndPose = Auto_Blue_Goal.creekEndPose.mirror();

        PathConstraints constraints = new PathConstraints(
                0.995,
                0.1,
                0.1,
                0.007,
                100,
                0.3,
                10,
                1);

        void CreatePaths(Follower follower) {

            follower.setConstraints(constraints);

            viewObelisk = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    startPose,
                                    viewObeliskPose
                            )
                    )
                    .setLinearHeadingInterpolation(startPose.getHeading(), viewObeliskPose.getHeading())
                    .setConstraints(constraints)
                    .build();

            scorePreloads = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    viewObeliskPose,
                                    scorePose1
                            )
                    )
                    .setLinearHeadingInterpolation(viewObeliskPose.getHeading(), scorePose1.getHeading())
                    .setConstraints(constraints)
                    .build();

            lineupPickup1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    scorePose1,
                                    startPickup1Control,
                                    startPickup1Pose
                            )
                    )
                    .setLinearHeadingInterpolation(scorePose1.getHeading(), startPickup1Pose.getHeading())
                    .setConstraints(constraints)
                    .setTranslationalConstraint(0.1)
                    .build();

            grabArtifact1Pickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(startPickup1Pose, stop1Pickup1Pose))
                    .setConstantHeadingInterpolation(startPickup1Pose.getHeading())
                    .setConstraints(constraints)
                    .setTimeoutConstraint(1300)
                    .build();

            grabArtifact2Pickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(stop1Pickup1Pose, stop2Pickup1Pose))
                    .setConstantHeadingInterpolation(startPickup1Pose.getHeading())
                    .setConstraints(constraints)
                    .setTimeoutConstraint(1800)
                    .build();

            grabArtifact3Pickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(stop2Pickup1Pose, endPickup1Pose))
                    .setConstantHeadingInterpolation(startPickup1Pose.getHeading())
                    .setConstraints(constraints)
                    .build();

            scorePickup1 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    endPickup1Pose,
                                    scorePickup1Control,
                                    scorePose1
                            )
                    )
                    .setLinearHeadingInterpolation(endPickup1Pose.getHeading(), scorePose1.getHeading())
                    .setConstraints(constraints)
                    .build();


            lineupPickup2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    scorePose1,
                                    startPickup2Control,
                                    startPickup2Pose
                            )
                    )
                    .setLinearHeadingInterpolation(scorePose1.getHeading(), startPickup2Pose.getHeading())
                    .setConstraints(constraints)
                    .build();

            grabArtifact1Pickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(startPickup2Pose, stop1Pickup2Pose))
                    .setConstantHeadingInterpolation(startPickup2Pose.getHeading())
                    .setConstraints(constraints)
                    .setTimeoutConstraint(1500)
                    .build();

            grabArtifact2Pickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(stop1Pickup2Pose, stop2Pickup2Pose))
                    .setConstantHeadingInterpolation(startPickup2Pose.getHeading())
                    .setConstraints(constraints)
                    .setTimeoutConstraint(2000)
                    .build();

            grabArtifact3Pickup2 = follower.pathBuilder()
                    .addPath(new BezierLine(stop2Pickup2Pose, endPickup2Pose))
                    .setConstantHeadingInterpolation(startPickup2Pose.getHeading())
                    .setConstraints(constraints)
                    .build();

            scorePickup2 = follower.pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    endPickup2Pose,
                                    scorePickup2Control,
                                    scorePose1
                            )
                    )
                    .setLinearHeadingInterpolation(endPickup2Pose.getHeading(), scorePose1.getHeading())
                    .setConstraints(constraints)
                    .build();

            park = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    scorePose1,
                                    endPose
                            )
                    )
                    .setLinearHeadingInterpolation(scorePose1.getHeading(), endPose.getHeading())
                    .setConstraints(constraints)
                    .build();

            creekPark = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    scorePose1,
                                    creekEndPose
                            )
                    )
                    .setLinearHeadingInterpolation(scorePose1.getHeading(), creekEndPose.getHeading())
                    .setConstraints(constraints)
                    .build();
        }
    }

    public static class Auto_Red_Wall {

        public PathChain lineupPickup1;

        public PathChain grabArtifact1Pickup1;
        public PathChain grabArtifact2Pickup1;
        public PathChain grabArtifact3Pickup1;

        public PathChain scorePickup1;
        public PathChain park;

        public Auto_Red_Wall(Follower follower) {
            CreatePaths(follower);
        }

        public static Pose startPose = Auto_Blue_Wall.startPose.mirror();

        protected static Pose startPickupPose1 = Auto_Blue_Wall.startPickupPose1.mirror();

        protected static Pose stop1Pickup1Pose = Auto_Blue_Wall.stop1Pickup1Pose.mirror();
        protected static Pose stop2Pickup1Pose = Auto_Blue_Wall.stop2Pickup1Pose.mirror();

        protected static Pose endPickup1Pose = Auto_Blue_Wall.endPickup1Pose.mirror();
        protected static Pose scorePickup1Pose = Auto_Blue_Wall.scorePickup1Pose.mirror();
        protected static Pose endPose = Auto_Blue_Wall.endPose.mirror();

        PathConstraints constraints = new PathConstraints(
                0.995,
                0.1,
                0.1,
                0.007,
                100,
                0.3,
                10,
                1);

        void CreatePaths(Follower follower) {

            follower.setConstraints(constraints);

            lineupPickup1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    startPose,
                                    startPickupPose1
                            )
                    )
                    .setLinearHeadingInterpolation(startPose.getHeading(), startPickupPose1.getHeading())
                    .setConstraints(constraints)
                    .build();

            grabArtifact1Pickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(startPickupPose1, stop1Pickup1Pose))
                    .setConstantHeadingInterpolation(startPickupPose1.getHeading())
                    .setConstraints(constraints)
                    .setTimeoutConstraint(1300)
                    .build();

            grabArtifact2Pickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(stop1Pickup1Pose, stop2Pickup1Pose))
                    .setConstantHeadingInterpolation(startPickupPose1.getHeading())
                    .setConstraints(constraints)
                    .setTimeoutConstraint(1800)
                    .build();

            grabArtifact3Pickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(stop2Pickup1Pose, endPickup1Pose))
                    .setConstantHeadingInterpolation(startPickupPose1.getHeading())
                    .setConstraints(constraints)
                    .build();

            scorePickup1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    endPickup1Pose,
                                    scorePickup1Pose
                            )
                    )
                    .setLinearHeadingInterpolation(endPickup1Pose.getHeading(), scorePickup1Pose.getHeading())
                    .setConstraints(constraints)
                    .build();

            park = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    scorePickup1Pose,
                                    endPose
                            )
                    )
                    .setLinearHeadingInterpolation(scorePickup1Pose.getHeading(), endPose.getHeading())
                    .setConstraints(constraints)
                    .build();
        }
    }
}
