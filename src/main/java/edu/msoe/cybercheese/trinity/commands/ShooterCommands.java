package edu.msoe.cybercheese.trinity.commands;

import edu.msoe.cybercheese.trinity.subsystems.shooter.Shooter;
import edu.msoe.cybercheese.trinity.subsystems.shooter.ShooterConstants;
import edu.msoe.cybercheese.trinity.subsystems.shooter.ShooterMath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.Supplier;

public class ShooterCommands {

    public static Command runVelocity(final Shooter shooter, final double velocity) {
        return Commands.run(() -> {
            shooter.runVelocity(velocity);
        }, shooter);
    }

    public static Command runTargetVelocity(final Shooter shooter, final Supplier<Pose2d> poseSupplier) {
        return Commands.run(() -> {
            final var velocity = ShooterMath.calculateTrajectoryFromRobot(poseSupplier.get());

            // we use negative numbers to signal invalid states
            if (velocity < 0) return;

            // TODO: rads per sec for our linear velocities
            shooter.runVelocity(velocity * ShooterConstants.WHEEL_RADIUS);
        }, shooter);
    }

//    public static Command shootWhenAvailable(final Shooter shooter) {
//        return Commands.run(() -> {
//            if (Math.abs(shooter.targetVelocity()) >= 0.05 && shooter.isAtSpeed()) {
//                // TODO: shoot
//            }
//        }, TODO);
//    }
}
