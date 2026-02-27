package edu.msoe.cybercheese.trinity.subsystems.hopper;

import edu.msoe.cybercheese.trinity.replay.IO;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface HopperIO extends IO<HopperIO.HopperInputs> {
    class HopperInputs implements LoggableInputs {
        public double velocity = 0.0;

        @Override
        public void toLog(LogTable table) {
            table.put("velocityRadPerSec", velocity);
        }

        @Override
        public void fromLog(LogTable table) {
            table.get("velocityRadPerSec", velocity);
        }
    }

    default void setVoltage(double volts) {}

    default void setVelocity(double velocity) {}
}
