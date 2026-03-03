package edu.msoe.cybercheese.trinity.subsystems.intake;

import edu.msoe.cybercheese.trinity.replay.IO;
import edu.msoe.cybercheese.trinity.util.hw.MotorIO;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface IntakeIO extends IO<IntakeIO.IntakeInputs> {

    class IntakeInputs implements LoggableInputs {

        MotorIO.MotorInputs roller = new MotorIO.MotorInputs();
        MotorIO.MotorInputs lower = new MotorIO.MotorInputs();

        @Override
        public void toLog(LogTable table) {
            table.put("roller", this.roller);
            table.put("lower", this.lower);
        }

        @Override
        public void fromLog(LogTable table) {
            this.roller = table.get("roller", new MotorIO.MotorInputs());
            this.lower = table.get("lower", new MotorIO.MotorInputs());
        }
    }

    default void setRollerOpenLoop(double output) {}

    default void setRollerVelocity(double velocityRadPerSec) {}

    default void setLowerOpenLoop(double output) {}

    default void setLowerPosition(double position) {}
}
