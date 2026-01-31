package edu.msoe.cybercheese.trinity.subsystems.shooter;

import static edu.msoe.cybercheese.trinity.subsystems.shooter.ShooterConstants.*;
import static edu.msoe.cybercheese.trinity.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ShooterIOSpark implements ShooterIO {
    // Hardware
    private final SparkMax spark;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController controller;

    public ShooterIOSpark() {
        // Initialize Hardware
        spark = new SparkMax(SHOOTER_MOTOR_ID, MotorType.kBrushless);
        encoder = spark.getEncoder();
        controller = spark.getClosedLoopController();

        // Create Configuration
        var config = new SparkMaxConfig();

        // General Config (Shooters usually Coast, not Brake)
        config.idleMode(IdleMode.kCoast)
                .smartCurrentLimit(SHOOTER_CURRENT_LIMIT)
                .voltageCompensation(12.0);

        // Encoder Config
        config.encoder
                .velocityConversionFactor(SHOOTER_VELOCITY_FACTOR) // RPM to Rad/s usually
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);

        // PID Config
        config.closedLoop.pid(SHOOTER_KP, 0.0, SHOOTER_KD).outputRange(-1, 1);

        // Signals Config (Optimize CAN bus usage)
        config.signals
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20) // High priority for velocity
                .primaryEncoderPositionAlwaysOn(false) // Position less relevant for shooter
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20); // Useful for checking for jams

        // Apply Configuration
        tryUntilOk(
                spark,
                5,
                () -> spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {
        sparkStickyFault = false;

        // Read Velocity (The main input for a shooter)
        ifOk(spark, encoder::getVelocity, (value) -> inputs.velocity = value);

        // Note: If you add appliedVolts/amps to inputs later, read them here:
        // ifOk(spark, spark::getAppliedOutput, (value) -> inputs.appliedVolts = value * spark.getBusVoltage());
        // ifOk(spark, spark::getOutputCurrent, (value) -> inputs.currentAmps = value);
    }

    @Override
    public void setShooterOpenLoop(double volts) {
        spark.setVoltage(volts);
    }

    @Override
    public void setShooterVelocity(double velocityRadPerSec) {
        // Calculate Feedforward
        double ffVolts = SHOOTER_KS * Math.signum(velocityRadPerSec) + SHOOTER_KV * velocityRadPerSec;

        // Set Reference
        controller.setSetpoint(
                velocityRadPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffVolts, ArbFFUnits.kVoltage);
    }
}
