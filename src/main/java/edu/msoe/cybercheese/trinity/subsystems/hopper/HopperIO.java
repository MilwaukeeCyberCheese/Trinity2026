package edu.msoe.cybercheese.trinity.subsystems.hopper;

import edu.msoe.cybercheese.trinity.replay.IO;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface HopperIO extends IO<HopperIO.HopperIOInputs> {
    public static class HopperIOInputs implements LoggableInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;

        @Override
        public void toLog(LogTable table) {
            table.put("positionRad", positionRad);
            table.put("velocityRadPerSec", velocityRadPerSec);
            table.put("appliedVolts", appliedVolts);
            table.put("currentAmps", currentAmps);
        }

        @Override
        public void fromLog(LogTable table) {
            table.get("positionRad", positionRad);
            table.get("velocityRadPerSec", velocityRadPerSec);
            table.get("appliedVolts", appliedVolts);
            table.get("currentAmps", currentAmps);
        }
    }

    public default void updateInputs(HopperIOInputs inputs) {}

    public default void setVoltage(double volts) {}
}
