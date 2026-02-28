package edu.msoe.cybercheese.trinity.subsystems.intake;

import static edu.msoe.cybercheese.trinity.subsystems.intake.IntakeConstants.*;
import static edu.msoe.cybercheese.trinity.util.SparkUtil.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;

public class IntakeIOSpark implements IntakeIO {

    private final SparkMax rollerSpark;
    private final RelativeEncoder rollerEncoder;
    private final SparkClosedLoopController rollerController;

    private final SparkMax lowerSpark;
    private final AbsoluteEncoder lowerEncoder;
    private final SparkClosedLoopController lowerController;

    public IntakeIOSpark() {
        this.rollerSpark = new SparkMax(ROLLER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        this.rollerEncoder = this.rollerSpark.getEncoder();
        this.rollerController = this.rollerSpark.getClosedLoopController();

        this.lowerSpark = new SparkMax(LOWER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
        this.lowerEncoder = this.lowerSpark.getAbsoluteEncoder();
        this.lowerController = this.lowerSpark.getClosedLoopController();

        // Create Configuration
        var rollerConfig = new SparkMaxConfig();

        rollerConfig
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .smartCurrentLimit(ROLLER_CURRENT_LIMIT)
                .voltageCompensation(12.0);

        // Encoder Config
        rollerConfig
                .encoder
                .velocityConversionFactor(ROLLER_VELOCITY_FACTOR) // RPM to Rad/s usually
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);

        // PID Config
        rollerConfig.closedLoop.pid(ROLLER_KP, 0.0, ROLLER_KD).outputRange(-1, 1);

        // Signals Config (Optimize CAN bus usage)
        rollerConfig.signals.primaryEncoderVelocityAlwaysOn(true).primaryEncoderPositionAlwaysOn(false);

        // Apply Configuration
        tryUntilOk(
                rollerSpark,
                5,
                () -> rollerSpark.configure(
                        rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        var lowerConfig = new SparkMaxConfig();
        lowerConfig
                .idleMode(SparkBaseConfig.IdleMode.kBrake)
                .smartCurrentLimit(LOWER_MOTOR_CURRENT_LIMIT)
                .voltageCompensation(12.0);
        lowerConfig
                .absoluteEncoder
                .positionConversionFactor(LOWER_ENCODER_POSITION_FACTOR)
                .velocityConversionFactor(LOWER_ENCODER_VELOCITY_FACTOR)
                .averageDepth(2);
        lowerConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
//                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
//                .positionWrappingEnabled(true)
//                .positionWrappingInputRange(LOWER_PID_MIN_INPUT, LOWER_PID_MAX_INPUT)
                .pid(LOWER_KP, 0.0, LOWER_KD);
//        lowerConfig.signals.absoluteEncoderPositionAlwaysOn(true).absoluteEncoderVelocityAlwaysOn(true);
        tryUntilOk(
                lowerSpark,
                5,
                () -> lowerSpark.configure(
                        lowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        sparkStickyFault = false;

        ifOk(this.rollerSpark, this.rollerEncoder::getVelocity, (value) -> inputs.rollerVelocity = value);

        sparkStickyFault = false;

        ifOk(this.lowerSpark, this.lowerEncoder::getPosition, (value) -> inputs.lowerPosition = value);
    }

    @Override
    public void setRollerOpenLoop(double output) {
        this.rollerSpark.setVoltage(output);
    }

    @Override
    public void setRollerVelocity(double velocityRadPerSec) {
        // Calculate Feedforward
        double ffVolts = ROLLER_KS * Math.signum(velocityRadPerSec) + ROLLER_KV * velocityRadPerSec;

        // Set Reference
        rollerController.setSetpoint(
                velocityRadPerSec,
                SparkBase.ControlType.kVelocity,
                ClosedLoopSlot.kSlot0,
                ffVolts,
                SparkClosedLoopController.ArbFFUnits.kVoltage);
    }

    @Override
    public void setLowerOpenLoop(double output) {
        this.lowerSpark.setVoltage(output);
    }

    @Override
    public void setLowerPosition(double velocityRadPerSec) {

        // Calculate Feedforward
        double ffVolts = ROLLER_KS * Math.signum(velocityRadPerSec) + ROLLER_KV * velocityRadPerSec;

        // Set Reference
        lowerController.setSetpoint(
                velocityRadPerSec,
                SparkBase.ControlType.kVelocity,
                ClosedLoopSlot.kSlot0,
                ffVolts,
                SparkClosedLoopController.ArbFFUnits.kVoltage);

//        double setpoint = MathUtil.inputModulus(position, LOWER_PID_MIN_INPUT, LOWER_PID_MAX_INPUT);
//        this.lowerController.setSetpoint(setpoint, SparkBase.ControlType.kPosition);
    }
}
