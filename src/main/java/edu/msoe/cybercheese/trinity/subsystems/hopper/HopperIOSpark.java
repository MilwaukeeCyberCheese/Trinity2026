package edu.msoe.cybercheese.trinity.subsystems.hopper;

import static edu.msoe.cybercheese.trinity.subsystems.hopper.HopperConstants.*;
import static edu.msoe.cybercheese.trinity.subsystems.intake.IntakeConstants.ROLLER_KS;
import static edu.msoe.cybercheese.trinity.subsystems.intake.IntakeConstants.ROLLER_KV;
import static edu.msoe.cybercheese.trinity.util.SparkUtil.*;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class HopperIOSpark implements HopperIO {

    private final SparkMax hopperSpark;
    private final RelativeEncoder hopperEncoder;
    private final SparkClosedLoopController hopperController;

    public HopperIOSpark() {
        this.hopperSpark = new SparkMax(HopperConstants.HOPPER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        this.hopperEncoder = this.hopperSpark.getEncoder();
        this.hopperController = this.hopperSpark.getClosedLoopController();

        var hopperConfig = new SparkMaxConfig();

        hopperConfig
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .smartCurrentLimit(HOPPER_CURRENT_LIMIT)
                .voltageCompensation(12.0);

        // Encoder Config
        hopperConfig
                .encoder
                .velocityConversionFactor(HOPPER_VELOCITY_FACTOR) // RPM to Rad/s usually
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);

        // PID Config
        hopperConfig.closedLoop.pid(HOPPER_KP, 0.0, HOPPER_KD).outputRange(-1, 1);

        // Signals Config (Optimize CAN bus usage)
        hopperConfig.signals.primaryEncoderVelocityAlwaysOn(true).primaryEncoderPositionAlwaysOn(false);

        // Apply Configuration
        tryUntilOk(
                hopperSpark,
                5,
                () -> hopperSpark.configure(
                        hopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    @Override
    public void updateInputs(HopperInputs inputs) {
        sparkStickyFault = false;

        ifOk(this.hopperSpark, this.hopperEncoder::getVelocity, (value) -> inputs.velocity = value);
    }

    public void setOpenLoop(double value) {
        this.hopperSpark.setVoltage(value);
    }

    public void setVelocity(double velocityRadPerSec) {
        // Calculate Feedforward
        double ffVolts = ROLLER_KS * Math.signum(velocityRadPerSec) + ROLLER_KV * velocityRadPerSec;

        System.out.println("hopper: " + velocityRadPerSec);

        // Set Reference
        hopperController.setSetpoint(
                velocityRadPerSec,
                SparkBase.ControlType.kVelocity,
                ClosedLoopSlot.kSlot0,
                ffVolts,
                SparkClosedLoopController.ArbFFUnits.kVoltage);
    }
}
