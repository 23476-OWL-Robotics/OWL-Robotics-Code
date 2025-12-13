package org.firstinspires.ftc.teamcode.Util.Paths;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;

public class TelePaths {

    public static class Tele_Blue {

        public Tele_Blue() {}

        protected static Pose pickupPose = new Pose(132, 12, Math.toRadians(0));
        protected static Pose scorePose1 = new Pose(32, 105, Math.toRadians(-45));
        protected static Pose scorePose2 = new Pose(80, 22, Math.toRadians(-55));
        protected static Pose parkPose = new Pose(105.3, 33.3, Math.toRadians(0));

        PathConstraints constraints = new PathConstraints(1, 0);

        public PathChain Pickup(Follower f) {

            return f.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    f.getPose(),
                                    pickupPose
                            )
                    )
                    .setLinearHeadingInterpolation(f.getHeading(), pickupPose.getHeading())
                    .setConstraints(constraints)
                    .build();
        }

        public PathChain Score1(Follower f) {

            return f.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    f.getPose(),
                                    scorePose1
                            )
                    )
                    .setLinearHeadingInterpolation(f.getHeading(), scorePose1.getHeading())
                    .setConstraints(constraints)
                    .build();
        }

        public PathChain Score2(Follower f) {

            return f.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    f.getPose(),
                                    scorePose2
                            )
                    )
                    .setLinearHeadingInterpolation(f.getHeading(), scorePose2.getHeading())
                    .setConstraints(constraints)
                    .build();
        }

        public PathChain Park(Follower f) {
            return f.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    f.getPose(),
                                    parkPose
                            )
                    )
                    .setLinearHeadingInterpolation(f.getHeading(), parkPose.getHeading())
                    .setConstraints(constraints)
                    .build();
        }
    }

    public static class Tele_Red {

        public Tele_Red() {}

        protected static Pose pickupPose = Tele_Blue.pickupPose.mirror();
        protected static Pose scorePose1 = Tele_Blue.scorePose1.mirror();
        protected static Pose scorePose2 = Tele_Blue.scorePose2.mirror();
        protected static Pose parkPose = Tele_Blue.parkPose.mirror();

        PathConstraints constraints = new PathConstraints(1, 0);

        public PathChain Pickup(Follower f) {

            return f.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    f.getPose(),
                                    pickupPose
                            )
                    )
                    .setLinearHeadingInterpolation(f.getHeading(), pickupPose.getHeading())
                    .setConstraints(constraints)
                    .build();
        }

        public PathChain Score1(Follower f) {

            return f.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    f.getPose(),
                                    scorePose1
                            )
                    )
                    .setLinearHeadingInterpolation(f.getHeading(), scorePose1.getHeading())
                    .setConstraints(constraints)
                    .build();
        }

        public PathChain Score2(Follower f) {

            return f.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    f.getPose(),
                                    scorePose2
                            )
                    )
                    .setLinearHeadingInterpolation(f.getHeading(), scorePose2.getHeading())
                    .setConstraints(constraints)
                    .build();
        }

        public PathChain Park(Follower f) {
            return f.pathBuilder()
                    .addPath(
                            new BezierLine(
                                    f.getPose(),
                                    parkPose
                            )
                    )
                    .setLinearHeadingInterpolation(f.getHeading(), parkPose.getHeading())
                    .setConstraints(constraints)
                    .build();
        }
    }
}
