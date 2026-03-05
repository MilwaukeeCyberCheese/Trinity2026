package edu.msoe.cybercheese.trinity.subsystems.intakeroller;

import edu.msoe.cybercheese.trinity.util.hw.MotorIO;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeRoller extends SubsystemBase {
    private final MotorIO.MotorInputs inputs = new MotorIO.MotorInputs();
    private final MotorIO roller;

    public IntakeRoller(MotorIO roller) {
        this.roller = roller;
    }

    @Override
    public void periodic() {
        this.roller.updateInputs(this.inputs);
        Logger.processInputs("IntakeRoller", this.inputs);

        if (DriverStation.isDisabled()) {
            this.roller.runOpenLoop(0);
        }
    }

    public void runVelocity(double velocity) {
        this.roller.runVelocity(velocity);
    }
}
