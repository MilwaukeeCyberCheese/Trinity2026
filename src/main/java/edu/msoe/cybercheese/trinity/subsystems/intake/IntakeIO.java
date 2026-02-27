package edu.msoe.cybercheese.trinity.subsystems.intake;

import edu.msoe.cybercheese.trinity.replay.IO;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface IntakeIO extends IO<IntakeIO.IntakeInputs> {

    class IntakeInputs implements LoggableInputs {

        double rollerVelocity;
        double lowerPosition;

        @Override
        public void toLog(LogTable table) {
            table.put("rollerVelocity", this.rollerVelocity);
            table.put("lowerPosition", this.lowerPosition);
        }

        @Override
        public void fromLog(LogTable table) {
            this.rollerVelocity = table.get("rollerVelocity", 0);
            this.lowerPosition = table.get("lowerPosition", 0);
        }
    }

    default void setRollerOpenLoop(double output) {}

    default void setRollerVelocity(double velocityRadPerSec) {}

    default void setLowerOpenLoop(double output) {}

    default void setLowerPosition(double position) {}
}
