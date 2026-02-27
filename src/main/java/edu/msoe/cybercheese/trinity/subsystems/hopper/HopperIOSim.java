package edu.msoe.cybercheese.trinity.subsystems.hopper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HopperIOSim implements HopperIO {
    private final DCMotorSim sim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    HopperConstants.HOPPER_GEARBOX, HopperConstants.HOPPER_MOI, HopperConstants.HOPPER_GEARING),
            HopperConstants.HOPPER_GEARBOX);

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
