package edu.msoe.cybercheese.trinity.util.hw;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;

public class MotorIOSim extends MotorIOSpark {

    private final SparkSim sim;

    public MotorIOSim(final MotorConfig config) {
        super(-1, config);
        this.sim = config.kind() == MotorConfig.ControllerKind.FLEX
                ? new SparkFlexSim((SparkFlex) this.spark(), config.gearbox())
                : new SparkMaxSim((SparkMax) this.spark(), config.gearbox());
    }

    @Override
    public void updateInputs(MotorInputs inputs) {
        //        this.sim.update(0.02);

        super.updateInputs(inputs);
    }
}
