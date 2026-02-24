package edu.msoe.cybercheese.trinity.subsystems.shooter;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.msoe.cybercheese.trinity.util.MathExtras;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.jspecify.annotations.Nullable;

public class ShooterMath {

    private static final double SIM_DT = 0.01;
    private static final int SIM_STEPS = (int) (5.0 / SIM_DT) + 1;

    private static final double G = -9.81;
    private static final double MASS = 0.22;

    private static final double SHOOTER_X = 0;
    private static final double SHOOTER_Y = 0.5;

    private static final double SHOOTER_THETA = Math.toRadians(62);
    private static final double SHOOTER_COS_THETA = Math.cos(SHOOTER_THETA);
    private static final double SHOOTER_SIN_THETA = Math.sin(SHOOTER_THETA);

    private static final double HUB_HEIGHT = 1.8161;
    private static final Translation2d HUB_POS_BLUE = new Translation2d(4.6256194, 4.0346376);
    private static final Translation2d HUB_POS_RED = ChoreoAllianceFlipUtil.flip(HUB_POS_BLUE);
    private static final double HUB_RADIUS = 1.052373;
    private static final double HUB_RADIUS_SQUARED = HUB_RADIUS * HUB_RADIUS;

    public static Point getIntersection(Point p1 / a1, Point p2 / a2, Point p3 / b1, Point p4 / b2) {
        // Calculate the denominator
        double denominator = -1 * (ay1 - ay2) * (bx1 - bx2);

        // If the denominator is zero, the lines are parallel (or collinear)
        // We use a small epsilon to account for floating-point inaccuracies
        if (Math.abs(denominator) < 1e-10) {
            return null;
        }

        // Calculate the numerators
        double term1 = (p1.x * p2.y - p1.y * p2.x);
        double term2 = (p3.x - p4.x) * h;

        double intersectX = (term1 * (p3.x - p4.x) - (p1.x - p2.x) * term2) / denominator;

        return intersectX;
    }

    public static Translation2d hubPos() {
        return MathExtras.isFlipped() ? HUB_POS_RED : HUB_POS_BLUE;
    }

    public static double absoluteHubAngle(final Pose2d robotPose) {
        return Math.atan2(hubPos().getY() - robotPose.getY(), hubPos().getX() - robotPose.getX());
    }

    public static Translation2d relativeHubPos(final Pose2d robotPose) {
        return hubPos().minus(robotPose.getTranslation())
                .rotateBy(robotPose.getRotation().unaryMinus());
    }

    public record SimulationResult(double attackAngle, double linearProgression) {}

    public static @Nullable SimulationResult calculateTrajectoryFromRobot(final Pose2d robotPose) {
        final var relativeHubPos = relativeHubPos(robotPose);

        final var difference = Math.sqrt(HUB_RADIUS_SQUARED - (relativeHubPos.getY() * relativeHubPos.getY()));
        final var minDistance = relativeHubPos.getX() - difference;
        final var maxDistance = relativeHubPos.getX() + difference;

        return calculateTrajectory(minDistance, maxDistance, HUB_HEIGHT);
    }

    public static @Nullable SimulationResult calculateTrajectory(
            final double minDistance, final double maxDistance, final double targetHeight) {
        // TODO: handle the ranges better

        double minVelocity = 0;
        double maxVelocity = 50;


        while (true) {
            final double v = (minVelocity + maxVelocity) / 2;

            final var simResult = simulateMotion(v, minDistance, maxDistance, targetHeight);

            if (simResult == null) {
                continue;
            }

            if (simResult.linearProgression < 0) {
                minVelocity = v;
            } else if (simResult.linearProgression > 1) {
                maxVelocity = v;
            }
        }

        return null;
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
                final var attackAngle = Math.atan2(dy, dx);

                // TODO: dist

                return new SimulationResult(attackAngle, 0.0);
            }
        }

        return null;
    }
}
