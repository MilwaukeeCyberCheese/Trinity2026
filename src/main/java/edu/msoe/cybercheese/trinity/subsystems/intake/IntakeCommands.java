package edu.msoe.cybercheese.trinity.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class IntakeCommands {

    public static Command runValues(final Intake intake, double position, double velocity) {
        return Commands.run(
                () -> {
//                    intake.setLowerPosition(position);
                    intake.setRollerVelocity(velocity);
                },
                intake);
    }
}
