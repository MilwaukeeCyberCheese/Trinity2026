package edu.msoe.cybercheese.trinity.subsystems.intake;

import edu.msoe.cybercheese.trinity.util.hw.MotorIO;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    private final MotorIO.MotorInputs inputs = new MotorIO.MotorInputs();
    private final MotorIO lift;

    public Intake(MotorIO lift) {
        this.lift = lift;
    }

    @Override
    public void periodic() {
        this.lift.updateInputs(this.inputs);
        Logger.processInputs("Intake", this.inputs);

        if (DriverStation.isDisabled()) {
            this.lift.runOpenLoop(0);
        }
    }

    public void setLowerPosition(final double position) {
        this.lift.runPosition(position);
    }
}
