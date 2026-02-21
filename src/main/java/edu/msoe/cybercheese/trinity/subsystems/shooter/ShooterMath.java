package edu.msoe.cybercheese.trinity.subsystems.shooter;

import org.jspecify.annotations.Nullable;

import java.util.Arrays;

public class ShooterMath {

    private static final double G = -9.81;

    private static final double SHOOTER_X = 0;
    private static final double SHOOTER_Y = 0.5;

    private static final double SHOOTER_THETA = Math.toRadians(62);
    private static final double SHOOTER_COS_THETA = Math.cos(SHOOTER_THETA);
    private static final double SHOOTER_SIN_THETA = Math.sin(SHOOTER_THETA);

    private static final double OFFSET_X = 4.6256194;
    private static final double OFFSET_Y = 4.0346376;

    public record SimulationResult(
        double v,
        double[] x,
        double[] y,
        double[] t
    ) {

    }

    public static @Nullable SimulationResult calculateTrajectoryFromRobot() {

    }

    public static @Nullable SimulationResult calculateTrajectory(final double targetX, final double targetY) {
        final double dv = 0.1;
        final double maxVelocity = 50;

        double v = 1;

        while (v <= maxVelocity) {
            final SimulationResult result = simulateMotion(v);

            for (int i = 0; i < result.x.length; i++) {
                if (result.x[i] >= targetX && result.y[i] >= targetY) {
                    return result;
                }
            }

            v += dv;
        }

        return null;
    }

    public static SimulationResult simulateMotion(double v) {
        final double m = 0.22;
        final double rho = 1.225;
        final double Cd = 0.47;
        final double A = 0.0176;

        final double dt = 0.01;
        final int N = (int) (5.0 / dt) + 1;

        double[] t = new double[N];
        for (int i = 0; i < N; i++) {
            t[i] = i * dt;
        }


        double[] x = new double[N];
        double[] y = new double[N];
        double[] vx = new double[N];
        double[] vy = new double[N];

        x[0] = SHOOTER_X;
        y[0] = SHOOTER_Y;
        vx[0] = v * SHOOTER_COS_THETA;
        vy[0] = v * SHOOTER_SIN_THETA;

        int actualSize = N;

        for (int i = 1; i < N; i++) {
            final double currentV = Math.hypot(vx[i - 1], vy[i - 1]);
            final double Fdx = -0.5 * rho * Cd * A * currentV * vx[i - 1];
            final double Fdy = -0.5 * rho * Cd * A * currentV * vy[i - 1];

            final double ax = Fdx / m;
            final double ay = (Fdy / m) + G;

            vx[i] = vx[i - 1] + ax * dt;
            vy[i] = vy[i - 1] + ay * dt;

            x[i] = x[i - 1] + vx[i] * dt;
            y[i] = y[i - 1] + vy[i] * dt;

            if (y[i] <= 0) {
                actualSize = i + 1;
                break;
            }
        }

        return new SimulationResult(
                v,
                Arrays.copyOf(x, actualSize),
                Arrays.copyOf(y, actualSize),
                Arrays.copyOf(t, actualSize)
        );
    }

}
