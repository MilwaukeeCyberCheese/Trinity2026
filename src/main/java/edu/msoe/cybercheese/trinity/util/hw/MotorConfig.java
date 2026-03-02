package edu.msoe.cybercheese.trinity.util.hw;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public record MotorConfig(
        ControllerKind kind,
        ControlMode mode,
        double sampleFrequency,
        boolean inverted,
        boolean encoderInverted,
        DCMotor gearbox,
        double gearing,
        int currentLimit,
        boolean brakeOnIdle,
        boolean positionWrapping,
        double moi,
        double p,
        double d,
        double kS,
        double kV,
        double simP,
        double simD,
        double simKS,
        double simKV) {

    private static final double POSITION_FACTOR = 2. * Math.PI;
    private static final double VELOCITY_FACTOR = 2. * Math.PI / 60.;

    public SparkBaseConfig sparkConfig() {
        final var config = this.kind == ControllerKind.FLEX ? new SparkFlexConfig() : new SparkMaxConfig();

        // TODO: inversion?
        config
                .inverted(this.inverted).idleMode(this.brakeOnIdle ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast)
                .smartCurrentLimit(this.currentLimit)
                .voltageCompensation(12.0);

        config.closedLoop
                .feedbackSensor(
                        this.mode == ControlMode.POSITION
                                ? FeedbackSensor.kAbsoluteEncoder
                                : FeedbackSensor.kPrimaryEncoder)
                .pid(this.p, 0.0, this.d);

        if (this.positionWrapping) {
            config.closedLoop.positionWrappingEnabled(true).positionWrappingInputRange(0, 2. * Math.PI);
        }

        if (this.mode == ControlMode.VELOCITY) {
            config.signals
                    .primaryEncoderPositionAlwaysOn(true)
                    .primaryEncoderPositionPeriodMs((int) (1000.0 / this.sampleFrequency))
                    .primaryEncoderVelocityAlwaysOn(true)
                    .primaryEncoderVelocityPeriodMs(20);
            config.encoder
                    .inverted(this.encoderInverted)
                    .positionConversionFactor(POSITION_FACTOR / this.gearing)
                    .velocityConversionFactor(VELOCITY_FACTOR / this.gearing)
                    .uvwMeasurementPeriod(10)
                    .uvwAverageDepth(2);
        } else {
            config.signals
                    .absoluteEncoderPositionAlwaysOn(true)
                    .absoluteEncoderPositionPeriodMs((int) (1000.0 / this.sampleFrequency))
                    .absoluteEncoderVelocityAlwaysOn(true)
                    .absoluteEncoderVelocityPeriodMs(20);
            config.absoluteEncoder
                    .inverted(this.encoderInverted)
                    .positionConversionFactor(POSITION_FACTOR / this.gearing)
                    .velocityConversionFactor(VELOCITY_FACTOR / this.gearing)
                    .averageDepth(2);
        }

        config.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);

        return config;
    }

    public DCMotorSim buildSim() {
        return new DCMotorSim(LinearSystemId.createDCMotorSystem(this.gearbox, this.moi, this.gearing), this.gearbox);
    }

    public enum ControllerKind {
        MAX,
        FLEX,
    }

    public enum ControlMode {
        POSITION,
        VELOCITY,
    }
}
