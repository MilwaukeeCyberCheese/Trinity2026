package edu.msoe.cybercheese.trinity.subsystems.loader;

import edu.msoe.cybercheese.trinity.replay.IO;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface LoaderIO extends IO<LoaderIO.LoaderInputs> {

    class LoaderInputs implements LoggableInputs {
        public double velocity = 0.0;
        // TODO: appliedVolts and appliedAmps

        @Override
        public void toLog(LogTable table) {
            table.put("shooterVelocity", this.velocity);
        }

        @Override
        public void fromLog(LogTable table) {
            this.velocity = table.get("shooterVelocity", this.velocity);
        }
    }

    default void setLoaderVelocity(double velocity) {}
}
