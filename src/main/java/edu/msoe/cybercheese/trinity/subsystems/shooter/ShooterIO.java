package edu.msoe.cybercheese.trinity.subsystems.shooter;

import edu.msoe.cybercheese.trinity.replay.IO;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ShooterIO extends IO<ShooterIO.ShooterInputs> {

    class ShooterInputs implements LoggableInputs {
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

    /** Run the shooter motor at the specified voltage. */
    default void setShooterOpenLoop(double volts) {}

    /** Run the shooter motor at the specified velocity in Rad/Sec. */
    default void setShooterVelocity(double velocityRadPerSec) {}
}
