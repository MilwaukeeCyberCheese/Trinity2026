package edu.msoe.cybercheese.trinity.commands;

import edu.msoe.cybercheese.trinity.subsystems.shooter.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ShooterCommands {

    public static Command shootIfAvailable(final Shooter shooter) {
        return Commands.runOnce()
    }
}
