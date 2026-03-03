package edu.msoe.cybercheese.trinity.util.hw;

import edu.msoe.cybercheese.trinity.replay.IO;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface MotorIO extends IO<MotorIO.MotorInputs> {

    class MotorInputs implements LoggableInputs {

        public boolean connected = false;
        public double position = 0.0;
        public double velocity = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;

        @Override
        public void toLog(LogTable table) {
            table.put("connected", this.connected);
            table.put("position", this.position);
            table.put("velocity", this.velocity);
            table.put("appliedVolts", this.appliedVolts);
            table.put("currentAmps", this.currentAmps);
        }

        @Override
        public void fromLog(LogTable table) {
            this.connected = table.get("connected", this.connected);
            this.position = table.get("position", this.position);
            this.velocity = table.get("velocity", this.velocity);
            this.appliedVolts = table.get("appliedVolts", this.appliedVolts);
            this.currentAmps = table.get("currentAmps", this.currentAmps);
        }
    }

    default void stop() {
        this.runOpenLoop(0);
    }

    default void runOpenLoop(double output) {}

    // TODO: doc that this is post gearbox rads/sec
    default void runVelocity(double velocity) {}

    default void runPosition(double position) {}
}
