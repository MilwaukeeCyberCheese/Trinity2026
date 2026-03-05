package edu.msoe.cybercheese.trinity.subsystems.intakeroller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class IntakeRollerCommands {

    public static Command runVelocity(final IntakeRoller roller, double velocity) {
        return Commands.run(
                () -> {
                    roller.runVelocity(velocity);
                },
                roller);
    }
}
