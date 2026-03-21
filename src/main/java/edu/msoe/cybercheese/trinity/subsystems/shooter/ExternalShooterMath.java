package edu.msoe.cybercheese.trinity.subsystems.shooter;

import org.jspecify.annotations.Nullable;

public class ExternalShooterMath {

    private static final double SIM_DT = 0.01;
    private static final int SIM_STEPS = (int) (5.0 / SIM_DT) + 1;

    private static final double G = -9.81;
    private static final double MASS = 0.22;

    private static final double SHOOTER_X = 0;
    private static final double SHOOTER_Y = 0.5;

    private static final double SHOOTER_THETA = Math.toRadians(62);
    private static final double SHOOTER_COS_THETA = Math.cos(SHOOTER_THETA);
    private static final double SHOOTER_SIN_THETA = Math.sin(SHOOTER_THETA);

    public record SimulationResult(double attackAngle, double linearProgression) {}

    public static double calculateTrajectory(
            final double minDistance, final double maxDistance, final double targetHeight) {
        // TODO: handle the ranges better

        double minVelocity = 0;
        double maxVelocity = 50;

        while (Math.abs(maxVelocity - minVelocity) > 0.2) {
            final double v = (minVelocity + maxVelocity) / 2;

            final var simResult = simulateMotion(v, minDistance, maxDistance, targetHeight);

            if (simResult == null) {
                // TODO: fix this lol

                // we sim 5 seconds, if we don't hit the ground we're probably doing something dumb
                maxVelocity = v;
                continue;
            }

            if (simResult.linearProgression < 0.1 /*|| simResult.attackAngle > -0.2*/) {
                minVelocity = v;
            } else if (simResult.linearProgression > 0.9 /*|| simResult.attackAngle < -1.5*/) {
                maxVelocity = v;
            } else {
                return v;
            }
        }

        return -Double.MAX_VALUE;
    }

    public static @Nullable SimulationResult simulateMotion(
            final double inputVelocity, final double minDistance, final double maxDistance, final double targetHeight) {
        final double rho = 1.225;
        final double Cd = 0.47;
        final double A = 0.0176;

        double x = SHOOTER_X;
        double y = SHOOTER_Y;
        double vx = inputVelocity * SHOOTER_COS_THETA;
        double vy = inputVelocity * SHOOTER_SIN_THETA;

        for (int i = 1; i < SIM_STEPS; i++) {
            final double currentV = Math.hypot(vx, vy);
            final double Fdx = -0.5 * rho * Cd * A * currentV * vx;
            final double Fdy = -0.5 * rho * Cd * A * currentV * vy;

            final double ax = Fdx / MASS;
            final double ay = (Fdy / MASS) + G;

            vx += ax * SIM_DT;
            vy += ay * SIM_DT;

            final var lastX = x;
            final var lastY = y;

            final var dx = vx * SIM_DT;
            final var dy = vy * SIM_DT;

            x += dx;
            y += dy;

            if (lastY >= targetHeight && y < targetHeight) {

                // I *could* properly compute the ray intersection but this is easy and dt=0.01 so this *should* be fine
                final var hitX = x;
                final var hitY = y;

                final var linearProgression = (hitX - minDistance) / (maxDistance - minDistance);

                if (hitX >= minDistance && hitX <= maxDistance) {
                    final var attackAngle = Math.atan2(dy, dx);

                    return new SimulationResult(attackAngle, linearProgression);
                } else {
                    return new SimulationResult(0.0f, linearProgression);
                }
            }
        }

        return null;
    }
}
