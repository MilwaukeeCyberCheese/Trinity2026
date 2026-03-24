package edu.msoe.cybercheese.trinity.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class IntakeCommands {

    public static Command runPosition(final Intake intake, double position) {
        return Commands.runOnce(() -> intake.setLowerPosition(position), intake);
    }
}
