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
        public PathChain grabPickup1;
        public PathChain scorePickup1;
        public PathChain park;

        public PathChain creekPark;

        public Auto_Blue_Goal(Follower follower) {
            CreatePaths(follower);
        }

        public static Pose startPose = new Pose(18, 124.5, Math.toRadians(-129));
        protected static Pose viewObeliskPose = new Pose(56, 125, Math.toRadians(235));
        protected static Pose scorePose1 = new Pose(40, 100, Math.toRadians(-45));
        protected static Pose startPickup1Pose = new Pose(46, 84, Math.toRadians(180));
        protected static Pose startPickup1Control = new Pose(50, 96);
        protected static Pose nibble1Pickup1Pose = new Pose(38, 84, Math.toRadians(180));
        protected static Pose nibble2Pickup1Pose = new Pose(34, 84, Math.toRadians(180));
        protected static Pose endPickup1Pose = new Pose(18, 84, Math.toRadians(180));
        protected static Pose scorePickup1Control = new Pose(38, 88);
        protected static Pose endPose = new Pose(25, 70, Math.toRadians(-90));

        protected static Pose creekEndPose = new Pose(56, 136, Math.toRadians(0));

        PathConstraints constraints = new PathConstraints(1, 0);

        void CreatePaths(Follower follower) {
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
                    .setLinearHeadingInterpolation(startPose.getHeading(), scorePose1.getHeading())
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
                    .build();

            grabPickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(startPickup1Pose, nibble1Pickup1Pose))
                    .setConstantHeadingInterpolation(startPickup1Pose.getHeading())
                    .setConstraints(constraints)

                    .addPath(new BezierLine(nibble1Pickup1Pose, startPickup1Pose))
                    .setConstantHeadingInterpolation(startPickup1Pose.getHeading())
                    .setConstraints(constraints)

                    .addPath(new BezierLine(startPickup1Pose, nibble2Pickup1Pose))
                    .setConstantHeadingInterpolation(startPickup1Pose.getHeading())
                    .setConstraints(constraints)

                    .addPath(new BezierLine(nibble2Pickup1Pose, nibble1Pickup1Pose))
                    .setConstantHeadingInterpolation(startPickup1Pose.getHeading())
                    .setConstraints(constraints)

                    .addPath(new BezierLine(nibble1Pickup1Pose, endPickup1Pose))
                    .setConstantHeadingInterpolation(startPickup1Pose.getHeading())
                    .setConstraints(constraints)

                    .build();

            scorePickup1 = follower.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    endPickup1Pose,
                                    scorePose1
                            )
                    )
                    .setLinearHeadingInterpolation(endPickup1Pose.getHeading(), scorePose1.getHeading())
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
        public PathChain grabPickup1;
        public PathChain scorePickup1;
        public PathChain park;

        public Auto_Blue_Wall(Follower follower) {
            CreatePaths(follower);
        }

        public static Pose startPose = new Pose(56, 8, Math.toRadians(-90));
        protected static Pose startPickupPose1 = new Pose(46, 34.5, Math.toRadians(180));
        protected static Pose nibble1Pickup1Pose = new Pose(39, 34.5, Math.toRadians(180));
        protected static Pose nibble2Pickup1Pose = new Pose(34.5, 34.5, Math.toRadians(180));
        protected static Pose endPickup1Pose = new Pose(24, 34.5, Math.toRadians(180));
        protected static Pose scorePickup1Pose = new Pose(56, 12, Math.toRadians(-90));
        protected static Pose endPose = new Pose(34, 12, Math.toRadians(-90));

        PathConstraints constraints = new PathConstraints(1, 0);

        void CreatePaths(Follower follower) {
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

            grabPickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(startPickupPose1, nibble1Pickup1Pose))
                    .setConstantHeadingInterpolation(startPickupPose1.getHeading())
                    .setConstraints(constraints)

                    .addPath(new BezierLine(nibble1Pickup1Pose, startPickupPose1))
                    .setConstantHeadingInterpolation(startPickupPose1.getHeading())
                    .setConstraints(constraints)

                    .addPath(new BezierLine(startPickupPose1, nibble2Pickup1Pose))
                    .setConstantHeadingInterpolation(startPickupPose1.getHeading())
                    .setConstraints(constraints)

                    .addPath(new BezierLine(nibble2Pickup1Pose, nibble1Pickup1Pose))
                    .setConstantHeadingInterpolation(startPickupPose1.getHeading())
                    .setConstraints(constraints)

                    .addPath(new BezierLine(nibble1Pickup1Pose, endPickup1Pose))
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
        public PathChain grabPickup1;
        public PathChain scorePickup1;
        public PathChain park;

        public PathChain creekPark;

        public Auto_Red_Goal(Follower follower) {
            CreatePaths(follower);
        }

        public static Pose startPose = Auto_Blue_Goal.startPose.mirror();
        protected static Pose viewObeliskPose = Auto_Blue_Goal.viewObeliskPose.mirror();
        protected static Pose scorePose1 = Auto_Blue_Goal.scorePose1.mirror();
        protected static Pose startPickup1Pose = Auto_Blue_Goal.startPickup1Pose.mirror();
        protected static Pose startPickup1Control = Auto_Blue_Goal.startPickup1Control.mirror();
        protected static Pose nibble1Pickup1Pose = Auto_Blue_Goal.nibble1Pickup1Pose.mirror();
        protected static Pose nibble2Pickup1Pose = Auto_Blue_Goal.nibble2Pickup1Pose.mirror();
        protected static Pose endPickup1Pose = Auto_Blue_Goal.endPickup1Pose.mirror();
        protected static Pose scorePickup1Control = Auto_Blue_Goal.scorePickup1Control.mirror();
        protected static Pose endPose = Auto_Blue_Goal.endPose.mirror();

        protected static Pose creekEndPose = Auto_Blue_Goal.creekEndPose.mirror();

        PathConstraints constraints = new PathConstraints(1, 0);

        void CreatePaths(Follower follower) {
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
                    .build();

            grabPickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(startPickup1Pose, nibble1Pickup1Pose))
                    .setConstantHeadingInterpolation(startPickup1Pose.getHeading())
                    .setConstraints(constraints)

                    .addPath(new BezierLine(nibble1Pickup1Pose, startPickup1Pose))
                    .setConstantHeadingInterpolation(startPickup1Pose.getHeading())
                    .setConstraints(constraints)

                    .addPath(new BezierLine(startPickup1Pose, nibble2Pickup1Pose))
                    .setConstantHeadingInterpolation(startPickup1Pose.getHeading())
                    .setConstraints(constraints)

                    .addPath(new BezierLine(nibble2Pickup1Pose, nibble1Pickup1Pose))
                    .setConstantHeadingInterpolation(startPickup1Pose.getHeading())
                    .setConstraints(constraints)

                    .addPath(new BezierLine(nibble1Pickup1Pose, endPickup1Pose))
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
        public PathChain grabPickup1;
        public PathChain scorePickup1;
        public PathChain park;

        public Auto_Red_Wall(Follower follower) {
            CreatePaths(follower);
        }

        public static Pose startPose = Auto_Blue_Wall.startPose.mirror();
        protected static Pose startPickupPose1 = Auto_Blue_Wall.startPickupPose1.mirror();
        protected static Pose nibble1Pickup1Pose = Auto_Blue_Wall.nibble1Pickup1Pose.mirror();
        protected static Pose nibble2Pickup1Pose = Auto_Blue_Wall.nibble2Pickup1Pose.mirror();
        protected static Pose endPickup1Pose = Auto_Blue_Wall.endPickup1Pose.mirror();
        protected static Pose scorePickup1Pose = Auto_Blue_Wall.scorePickup1Pose.mirror();
        protected static Pose endPose = Auto_Blue_Wall.endPose.mirror();

        PathConstraints constraints = new PathConstraints(1, 0);

        void CreatePaths(Follower follower) {
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

            grabPickup1 = follower.pathBuilder()
                    .addPath(new BezierLine(startPickupPose1, nibble1Pickup1Pose))
                    .setConstantHeadingInterpolation(startPickupPose1.getHeading())
                    .setConstraints(constraints)

                    .addPath(new BezierLine(nibble1Pickup1Pose, startPickupPose1))
                    .setConstantHeadingInterpolation(startPickupPose1.getHeading())
                    .setConstraints(constraints)

                    .addPath(new BezierLine(startPickupPose1, nibble2Pickup1Pose))
                    .setConstantHeadingInterpolation(startPickupPose1.getHeading())
                    .setConstraints(constraints)

                    .addPath(new BezierLine(nibble2Pickup1Pose, nibble1Pickup1Pose))
                    .setConstantHeadingInterpolation(startPickupPose1.getHeading())
                    .setConstraints(constraints)

                    .addPath(new BezierLine(nibble1Pickup1Pose, endPickup1Pose))
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
