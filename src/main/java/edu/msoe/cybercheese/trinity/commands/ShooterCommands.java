package edu.msoe.cybercheese.trinity.commands;

import edu.msoe.cybercheese.trinity.subsystems.shooter.Shooter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class ShooterCommands {

    public static Command runVelocity(final Shooter shooter, final double velocity) {
        return Commands.run(
                () -> {
                    shooter.runVelocity(velocity);
                },
                shooter);
    }

    public static Command runTargetVelocity(final Shooter shooter, final Supplier<Pose2d> poseSupplier) {
        return Commands.run(() -> shooter.runAiming(poseSupplier.get()), shooter);
    }

    public static Command hailMary(final Shooter shooter) {
        return Commands.run(() -> shooter.runCharacterization(-13.5), shooter); // will this brownout? maybe! yolo!
    }

    public static Command runDefaultedVelocity(
            final Shooter shooter,
            final Supplier<Pose2d> poseSupplier,
            final double defaultVelocity,
            final BooleanSupplier shouldAim) {
        return Commands.run(
                () -> {
                    System.out.println("rdv: " + shouldAim.getAsBoolean());
                    if (shouldAim.getAsBoolean()) {
                        shooter.runAiming(poseSupplier.get());
                    } else {
                        shooter.setTargetLocked(true);
                        shooter.runVelocity(defaultVelocity);
                    }
                },
                shooter);
    }

    //    public static Command shootWhenAvailable(final Shooter shooter) {
    //        return Commands.run(() -> {
    //            if (Math.abs(shooter.isAtSpeed()) {
    //                // TODO: shoot
    //            }
    //        }, TODO);
    //    }
}
