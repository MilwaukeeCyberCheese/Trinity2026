package edu.msoe.cybercheese.trinity.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class HopperCommands {

    public static Command runVelocity(final Hopper hopper, final double velocity) {
        return Commands.run(() -> hopper.setVelocity(velocity), hopper);
    }
}
