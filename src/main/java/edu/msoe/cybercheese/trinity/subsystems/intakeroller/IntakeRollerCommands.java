package edu.msoe.cybercheese.trinity.subsystems.intakeroller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;

public class IntakeRollerCommands {

    public static Command runVelocity(final IntakeRoller roller, double velocity) {
        return runVelocity(roller, () -> velocity);
    }

    public static Command runVelocity(final IntakeRoller roller, final DoubleSupplier velocity) {
        return Commands.run(() -> roller.runVelocity(velocity.getAsDouble()), roller);
    }
}
