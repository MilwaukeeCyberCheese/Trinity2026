package edu.msoe.cybercheese.trinity.subsystems.loader;

import edu.msoe.cybercheese.trinity.subsystems.shooter.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class LoaderCommands {

    public static Command runVelocity(final Loader loader, double velocity) {
        return Commands.run(() -> loader.runVelocity(velocity), loader);
    }

    public static Command shootWhenReady(final Loader loader, final Shooter shooter, double velocity) {
        return Commands.run(
                () -> {
                    if (shooter.isAtSpeed()) {
                        loader.runVelocity(velocity);
                    }
                },
                loader);
    }
}
