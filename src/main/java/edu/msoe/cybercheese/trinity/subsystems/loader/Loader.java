package edu.msoe.cybercheese.trinity.subsystems.loader;

import edu.msoe.cybercheese.trinity.util.hw.MotorIO;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Loader extends SubsystemBase {

    private final MotorIO.MotorInputs inputs = new MotorIO.MotorInputs();
    private final MotorIO io;

    public Loader(MotorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        this.io.updateInputs(this.inputs);
        Logger.processInputs("Loader", this.inputs);
    }

    public void runVelocity(double velocity) {
        this.io.runVelocity(velocity);
    }
}
