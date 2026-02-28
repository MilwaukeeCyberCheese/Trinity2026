package edu.msoe.cybercheese.trinity.subsystems.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    private final IntakeIO.IntakeInputs inputs = new IntakeIO.IntakeInputs();
    private final IntakeIO intakeIo;

    public Intake(IntakeIO intakeIo) {
        this.intakeIo = intakeIo;
    }

    @Override
    public void periodic() {
        this.intakeIo.updateInputs(this.inputs);
        Logger.processInputs("Intake", this.inputs);

        if (DriverStation.isDisabled()) {
            this.intakeIo.setRollerOpenLoop(0);
            this.intakeIo.setLowerOpenLoop(0);
        }
    }

    public void setRollerVelocity(final double velocity) {
        this.intakeIo.setRollerVelocity(velocity);
    }

    public void setLowerPosition(final double position) {
        this.intakeIo.setLowerPosition(position);
    }
}
