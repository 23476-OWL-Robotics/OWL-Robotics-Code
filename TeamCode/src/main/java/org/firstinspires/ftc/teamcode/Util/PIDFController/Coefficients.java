package org.firstinspires.ftc.teamcode.Util.PIDFController;

public class Coefficients {

    public static class PositionCoefficients {
        double p;
        double i;
        double d;
        double f;

        double conversionUnit;

        public PositionCoefficients(double p, double i, double d, double f, double conversionUnit) {
            this.p = p;
            this.i = i;
            this.d = d;
            this.f = f;

            this.conversionUnit = conversionUnit;
        }

        public static class ExampleCoefficients extends PositionCoefficients {

            public ExampleCoefficients() {
                super(
                        0.0015,
                        0,
                        0.0,
                        0.0,
                        0.002583979328165375);
            }
        }

        public static class LiftMotorCoefficients extends PositionCoefficients {

            public LiftMotorCoefficients() {
                super(
                        0.007,
                        0.0,
                        0.0000012,
                        0.01,
                        0.011560694);
            }
        }
    }

    public static class VelocityCoefficients {
        double p;
        double i;
        double d;

        double ticksPerRev;

        public VelocityCoefficients(double p, double i, double d, double ticksPerRev) {
            this.p = p;
            this.i = i;
            this.d = d;

            this.ticksPerRev = ticksPerRev;
        }

        public static class ExampleCoefficients extends VelocityCoefficients {

            public ExampleCoefficients() {
                super(0.0,
                      0.0,
                      0.0,
                      28);
            }
        }

        public static class LauncherMotorCoefficients extends VelocityCoefficients {

            public LauncherMotorCoefficients() {
                super(0.00001,
                      0.0,
                      0.0,
                      28);
            }
        }

        public static class IntakeMotorCoefficients extends VelocityCoefficients {

            public IntakeMotorCoefficients() {
                super(
                        0.0,
                        0.0,
                        0.0,
                        145.1);
            }
        }
    }
}
