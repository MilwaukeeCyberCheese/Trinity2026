package edu.msoe.cybercheese.trinity.subsystems.shooter;

import static edu.msoe.cybercheese.trinity.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
    private final FlywheelSim flywheelSim;
    private final PIDController controller;

    private boolean closedLoop = false;
    private double ffVolts = 0.0;
    private double appliedVolts = 0.0;

    public ShooterIOSim() {
        // Create flywheel sim model
        flywheelSim = new FlywheelSim(
                LinearSystemId.createFlywheelSystem(SHOOTER_GEARBOX, SHOOTER_MOI, SHOOTER_GEARING),
                SHOOTER_GEARBOX,
                SHOOTER_GEARING);

        controller = new PIDController(SHOOTER_SIM_P, 0.0, SHOOTER_SIM_D);
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {
        // Run closed-loop control
        if (closedLoop) {
            // Calculate PID + FF
            appliedVolts = ffVolts + controller.calculate(flywheelSim.getAngularVelocityRadPerSec());
        } else {
            controller.reset();
        }

        // Clamp voltage and update physics
        appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
        flywheelSim.setInputVoltage(appliedVolts);
        flywheelSim.update(0.02);

        // Update inputs
        inputs.velocity = flywheelSim.getAngularVelocityRadPerSec();
    }

    @Override
    public void setShooterOpenLoop(double volts) {
        closedLoop = false;
        appliedVolts = volts;
    }

    @Override
    public void setShooterVelocity(double velocityRadPerSec) {
        closedLoop = true;
        // Calculate Feedforward (Simple kS + kV for velocity)
        ffVolts = SHOOTER_SIM_KS * Math.signum(velocityRadPerSec) + SHOOTER_SIM_KV * velocityRadPerSec;
        controller.setSetpoint(velocityRadPerSec);
    }
}
