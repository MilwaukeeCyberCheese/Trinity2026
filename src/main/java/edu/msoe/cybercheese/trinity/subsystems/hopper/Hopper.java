package edu.msoe.cybercheese.trinity.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
    private final HopperIO io;

    public Hopper(HopperIO io) {
        this.io = io;
    }

    public void setVoltage(double volts) {
        io.setVoltage(volts);
    }

    public Command runHopper(double volts) {
        return this.run(() -> setVoltage(volts));
    }

    public Command stop() {
        return this.runOnce(() -> setVoltage(0));
    }
}
