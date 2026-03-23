package edu.msoe.cybercheese.trinity.subsystems.shooter;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.msoe.cybercheese.trinity.util.MathExtras;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ShooterMath {

    private static final double HUB_HEIGHT = 1.8161;
    private static final double RELEASE_HEIGHT = 0.440;
    private static final double HUB_HEIGHT_OFFSET = HUB_HEIGHT - RELEASE_HEIGHT;

    private static final double SHOOT_OFFSET = -0.211;

    private static final Translation2d HUB_POS_BLUE = new Translation2d(4.6256194, 4.0346376);
    private static final Translation2d HUB_POS_RED = ChoreoAllianceFlipUtil.flip(HUB_POS_BLUE);
    private static final double HUB_RADIUS = 1.052373 / 2.;
    private static final double HUB_RADIUS_SQUARED = HUB_RADIUS * HUB_RADIUS;

    public static Translation2d hubPos() {
        return MathExtras.isFlipped() ? HUB_POS_RED : HUB_POS_BLUE;
    }

    public static double absoluteHubAngle(final Pose2d robotPose) {
        return Math.atan2(hubPos().getY() - robotPose.getY(), hubPos().getX() - robotPose.getX());
    }

    public static Translation2d relativeHubPos(final Pose2d robotPose) {
        final var shooterTranslation = robotPose.getTranslation().plus(new Translation2d(SHOOT_OFFSET, 0.));

        return hubPos().minus(shooterTranslation)
                .rotateBy(robotPose.getRotation().unaryMinus());
    }

    public static double calculateTrajectoryFromRobot(final Pose2d robotPose) {
        final var relativeHubPos = relativeHubPos(robotPose);

        final var difference = Math.sqrt(HUB_RADIUS_SQUARED - (relativeHubPos.getY() * relativeHubPos.getY()));
        final var minDistance = relativeHubPos.getX() - difference;
        final var maxDistance = relativeHubPos.getX() + difference;

        return ExternalShooterMath.calculateTrajectory(minDistance, maxDistance, HUB_HEIGHT_OFFSET);
    }
}
