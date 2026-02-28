package edu.msoe.cybercheese.trinity.subsystems.hopper;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {

    private final HopperIO.HopperInputs inputs = new HopperIO.HopperInputs();
    private final HopperIO io;

    public Hopper(HopperIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        this.io.updateInputs(this.inputs);
        Logger.processInputs("Hopper", this.inputs);

        if (DriverStation.isDisabled()) {
            this.io.setVoltage(0);
        }
    }

    public void setVelocity(double velocity) {
        this.io.setVelocity(velocity);
    }
}
