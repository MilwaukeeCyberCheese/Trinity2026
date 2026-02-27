package edu.msoe.cybercheese.trinity.subsystems.loader;

import static edu.msoe.cybercheese.trinity.subsystems.loader.LoaderConstants.*;
import static edu.msoe.cybercheese.trinity.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class LoaderIOSpark implements LoaderIO {

    private final SparkMax spark;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController controller;

    public LoaderIOSpark() {
        // Initialize Hardware
        spark = new SparkMax(LOADER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        encoder = spark.getEncoder();
        controller = spark.getClosedLoopController();

        // Create Configuration
        var config = new SparkMaxConfig();

        // General Config (Loaders usually Coast, not Brake)
        config.idleMode(SparkBaseConfig.IdleMode.kCoast)
                .smartCurrentLimit(LOADER_CURRENT_LIMIT)
                .voltageCompensation(12.0);

        // Encoder Config
        config.encoder
                .velocityConversionFactor(LOADER_VELOCITY_FACTOR) // RPM to Rad/s usually
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);

        // PID Config
        config.closedLoop.pid(LOADER_KP, 0.0, LOADER_KD).outputRange(-1, 1);

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
    public void updateInputs(LoaderIO.LoaderInputs inputs) {
        sparkStickyFault = false;

        // Read Velocity (The main input for a shooter)
        ifOk(spark, encoder::getVelocity, (value) -> inputs.velocity = value);

        // Note: If you add appliedVolts/amps to inputs later, read them here:
        // ifOk(spark, spark::getAppliedOutput, (value) -> inputs.appliedVolts = value * spark.getBusVoltage());
        // ifOk(spark, spark::getOutputCurrent, (value) -> inputs.currentAmps = value);
    }

    @Override
    public void setLoaderVelocity(double velocityRadPerSec) {
        // Calculate Feedforward
        double ffVolts = LOADER_KS * Math.signum(velocityRadPerSec) + LOADER_KV * velocityRadPerSec;

        // Set Reference
        controller.setSetpoint(
                velocityRadPerSec,
                SparkBase.ControlType.kVelocity,
                ClosedLoopSlot.kSlot0,
                ffVolts,
                SparkClosedLoopController.ArbFFUnits.kVoltage);
    }
}
