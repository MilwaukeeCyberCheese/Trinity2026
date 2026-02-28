package edu.msoe.cybercheese.trinity.util.hw;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public record MotorConfig(
        int canId,
        DCMotor gearbox,
        double gearing,
        double moi,
        double currentLimit,
        double positionFactor,
        double velocityFactor,
        double p,
        double d,
        double kS,
        double kV,
        double simP,
        double simD,
        double simKS,
        double simKV) {

    public DCMotorSim buildSim() {
        return new DCMotorSim(LinearSystemId.createDCMotorSystem(this.gearbox, this.moi, this.gearing), this.gearbox);
    }

    public enum ControlMode {
        POSITION,

    }
}
