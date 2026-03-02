package edu.msoe.cybercheese.trinity.util.hw;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class MotorIOSim implements MotorIO {

    private final MotorConfig config;

    private final DCMotorSim sim;
    private final PIDController positionController;

    public MotorIOSim(MotorConfig config) {
        this.config = config;
        this.sim = config.buildSim();
        this.positionController = new PIDController(config.simP(), 0., config.simD());
    }

    @Override
    public void runOpenLoop(double output) {
        this.sim.setInputVoltage(output);
    }

    @Override
    public void runVelocity(double velocity) {

    }

    @Override
    public void runPosition(double position) {

    }

    @Override
    public void updateInputs(MotorInputs inputs) {
        this.sim.update(0.02);

        inputs.connected = true;
        inputs.position = this.sim.getAngularPositionRad();
        inputs.velocity = this.sim.getAngularVelocityRadPerSec();
        inputs.appliedVolts = this.sim.getInputVoltage();
        inputs.currentAmps = this.sim.getCurrentDrawAmps();
    }

    public class HopperIOSim implements HopperIO {

        private double appliedVolts = 0.0;

        @Override
        public void updateInputs(HopperInputs inputs) {
            sim.update(0.02);
            inputs.velocity = sim.getAngularVelocityRadPerSec();
        }

        @Override
        public void setVoltage(double volts) {
            appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
            sim.setInputVoltage(appliedVolts);
        }
    }

}
