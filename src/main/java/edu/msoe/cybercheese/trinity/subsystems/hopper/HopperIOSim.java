package edu.msoe.cybercheese.trinity.subsystems.hopper;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HopperIOSim implements HopperIO {
    private final DCMotorSim sim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    HopperConstants.HOPPER_GEARBOX, HopperConstants.MOI_KG_M2, HopperConstants.GEAR_RATIO),
            HopperConstants.HOPPER_GEARBOX);

    private double appliedVolts = 0.0;

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        sim.update(0.02);
        inputs.positionRad = sim.getAngularPositionRad();
        inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = sim.getCurrentDrawAmps();
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
        sim.setInputVoltage(appliedVolts);
    }
}
