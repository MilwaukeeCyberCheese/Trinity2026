package edu.msoe.cybercheese.trinity.subsystems.intake;

import static edu.msoe.cybercheese.trinity.subsystems.intake.IntakeConstants.*;
import static edu.msoe.cybercheese.trinity.util.SparkUtil.*;

import com.revrobotics.spark.*;
import edu.msoe.cybercheese.trinity.util.hw.MotorIO;

public class IntakeIOImpl implements IntakeIO {

    private final MotorIO roller;
    private final MotorIO lower;

    public IntakeIOImpl(final MotorIO roller, final MotorIO lower) {
        this.roller = roller;
        this.lower = lower;
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        this.roller.updateInputs(inputs.roller);
        this.lower.updateInputs(inputs.lower);
    }

    @Override
    public void setRollerOpenLoop(double output) {
        this.roller.runOpenLoop(output);
    }

    @Override
    public void setRollerVelocity(double velocityRadPerSec) {
        this.roller.runVelocity(velocityRadPerSec);
    }

    @Override
    public void setLowerOpenLoop(double output) {
        this.lower.runOpenLoop(output);
    }

    @Override
    public void setLowerPosition(double position) {
        this.lower.runPosition(position);
    }
}
