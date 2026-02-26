package edu.msoe.cybercheese.trinity.subsystems.shooter;

import edu.msoe.cybercheese.trinity.subsystems.shooter.ShooterIO.ShooterInputs;
import edu.msoe.cybercheese.trinity.util.UnitTypes;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

    private final ShooterIO io;
    private final ShooterInputs inputs = new ShooterInputs();
    private final SysIdRoutine sysId;

    private double targetVelocity = 0.0;

    public Shooter(ShooterIO io) {
        this.io = io;

        // Configure SysId for the Shooter
        this.sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null, (state) -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism((voltage) -> runCharacterization(voltage.in(UnitTypes.VOLTS)), null, this));
    }

    @Override
    public void periodic() {
        this.io.updateInputs(this.inputs);
        Logger.processInputs("Shooter", this.inputs);

        if (DriverStation.isDisabled()) {
            this.stop();
        }

        Logger.recordOutput("Shooter/SetpointVelocity", targetVelocity);
        Logger.recordOutput("Shooter/Error", targetVelocity - inputs.velocity);
        Logger.recordOutput("Shooter/AtSpeed", isAtSpeed());
    }

    public double targetVelocity() {
        return targetVelocity;
    }

    /**
     * Runs the shooter at the desired velocity.
     *
     * @param velocityRadPerSec Velocity in radians/sec
     */
    public void runVelocity(double velocityRadPerSec) {
        this.targetVelocity = velocityRadPerSec;
        this.io.setShooterVelocity(velocityRadPerSec);
    }

    /** Runs the shooter with the specified voltage output. */
    public void runCharacterization(double volts) {
        // Characterization runs open loop
        this.targetVelocity = 0.0;
        this.io.setShooterOpenLoop(volts);
    }

    /** Stops the shooter. */
    public void stop() {
        this.targetVelocity = 0.0;
        this.io.setShooterOpenLoop(0.0);
    }

    /**
     * Checks if the shooter is at the target velocity within a tolerance.
     */
    public boolean isAtSpeed() {
        return Math.abs(inputs.velocity - targetVelocity) < 5.0 && targetVelocity >= 0.1;
    }

    /**
     * Returns the current velocity of the shooter.
     */
    @AutoLogOutput(key = "Shooter/MeasuredVelocity")
    public double getVelocity() {
        return inputs.velocity;
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }

    /**
     * Creates a command to spin the shooter to a specific velocity.
     */
    public Command spinTo(double velocityRadPerSec) {
        return this.run(() -> this.runVelocity(velocityRadPerSec));
    }
}
