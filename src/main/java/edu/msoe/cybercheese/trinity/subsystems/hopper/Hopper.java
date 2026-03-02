package edu.msoe.cybercheese.trinity.subsystems.hopper;

import edu.msoe.cybercheese.trinity.util.hw.MotorIO;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {

    private final MotorIO.MotorInputs inputs = new MotorIO.MotorInputs();
    private final MotorIO io;

    public Hopper(final MotorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        this.io.updateInputs(this.inputs);
        Logger.processInputs("Hopper", this.inputs);

        if (DriverStation.isDisabled()) {
            this.io.stop();
        }
    }

    public void setVelocity(double velocity) {
        this.io.runVelocity(velocity);
    }
}
