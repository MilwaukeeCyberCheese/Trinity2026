package edu.msoe.cybercheese.trinity.subsystems.loader;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Loader extends SubsystemBase {

    private final LoaderIO.LoaderInputs inputs = new LoaderIO.LoaderInputs();
    private final LoaderIO io;

    public Loader(LoaderIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        this.io.updateInputs(this.inputs);
        Logger.processInputs("Loader", this.inputs);
    }

    public void runVelocity(double velocity) {
        this.io.setLoaderVelocity(velocity);
    }
}
