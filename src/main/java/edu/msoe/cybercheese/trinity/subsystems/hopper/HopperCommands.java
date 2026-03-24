package edu.msoe.cybercheese.trinity.subsystems.hopper;

import edu.msoe.cybercheese.trinity.subsystems.shooter.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;

public class HopperCommands {

    public static Command runVelocity(final Hopper hopper, final DoubleSupplier velocity) {
        return Commands.run(() -> hopper.setVelocity(velocity.getAsDouble()), hopper);
    }

    public static Command shootWhenReady(final Hopper hopper, final Shooter shooter, final DoubleSupplier velocity) {
        return Commands.run(
                () -> {
                    if (shooter.isTargetLocked() && shooter.isAtSpeed()) {
                        hopper.setVelocity(velocity.getAsDouble());
                    }
                },
                hopper);
    }
}
