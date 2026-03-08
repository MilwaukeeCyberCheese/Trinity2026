package edu.msoe.cybercheese.trinity.util.hw;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.msoe.cybercheese.trinity.util.FFConstants;
import edu.msoe.cybercheese.trinity.util.PIDConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import java.util.Objects;
import org.jspecify.annotations.Nullable;

public record MotorConfig(
        ControllerKind kind,
        ControlMode mode,
        double sampleFrequency,
        boolean inverted,
        boolean encoderInverted,
        DCMotor gearbox,
        double gearing,
        @Nullable Double encoderGearing,
        int currentLimit,
        boolean brakeOnIdle,
        boolean positionWrapping,
        double moi,
        PIDConstants pid,
        FFConstants feedForward) {

    public static final double DEFAULT_SAMPLE_FREQUENCY = 50.0;

    public static final int VORTEX_CURRENT_LIMIT = 50;

    private static final double POSITION_FACTOR = 2. * Math.PI;
    private static final double VELOCITY_FACTOR = 2. * Math.PI / 60.;

    public SparkBaseConfig sparkConfig() {
        final var config = this.kind == ControllerKind.FLEX ? new SparkFlexConfig() : new SparkMaxConfig();

        config.inverted(this.inverted)
                .idleMode(this.brakeOnIdle ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast)
                .smartCurrentLimit(this.currentLimit)
                .voltageCompensation(12.0);

        config.closedLoop
                .feedbackSensor(
                        this.mode == ControlMode.POSITION
                                ? FeedbackSensor.kAbsoluteEncoder
                                : FeedbackSensor.kPrimaryEncoder)
                .pid(this.pid.p(), this.pid.i(), this.pid.d())
                .outputRange(-1, 1);

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
                    // .inverted(this.encoderInverted)
                    .positionConversionFactor(POSITION_FACTOR / this.effectiveEncoderGearing())
                    .velocityConversionFactor(VELOCITY_FACTOR / this.effectiveEncoderGearing())
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
                    .positionConversionFactor(POSITION_FACTOR / this.effectiveEncoderGearing())
                    .velocityConversionFactor(VELOCITY_FACTOR / this.effectiveEncoderGearing())
                    .averageDepth(2);
        }

        config.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);

        return config;
    }

    public DCMotorSim buildSim() {
        return new DCMotorSim(LinearSystemId.createDCMotorSystem(this.gearbox, this.moi, this.gearing), this.gearbox);
    }

    private double effectiveEncoderGearing() {
        return Objects.requireNonNullElse(this.encoderGearing, this.gearing);
    }

    public enum ControllerKind {
        MAX,
        FLEX,
    }

    public enum ControlMode {
        POSITION,
        VELOCITY,
    }

    private MotorConfig(final Builder builder) {
        this(
                builder.kind,
                builder.mode,
                builder.sampleFrequency,
                builder.inverted,
                builder.encoderInverted,
                builder.gearbox,
                builder.gearing,
                builder.encoderGearing,
                builder.currentLimit,
                builder.brakeOnIdle,
                builder.positionWrapping,
                builder.moi,
                builder.pid,
                builder.feedForward);
    }

    public static Builder builder(final ControllerKind kind, final ControlMode mode, final DCMotor gearbox) {
        return new Builder(kind, mode, gearbox);
    }

    public static Builder vortexBuilder(final ControllerKind kind, final ControlMode mode) {
        return new Builder(kind, mode, DCMotor.getNeoVortex(1))
                .currentLimit(50)
                .moi(0.002)
                .feedForward(new FFConstants(0.15, 0.019));
    }

    public static class Builder {
        private final ControllerKind kind;
        private final ControlMode mode;
        private final DCMotor gearbox;

        private double sampleFrequency = DEFAULT_SAMPLE_FREQUENCY;
        private boolean inverted = false;
        private boolean encoderInverted = false;
        private double gearing = 1.0;
        private @Nullable Double encoderGearing;
        private int currentLimit = 50;
        private boolean brakeOnIdle = true;
        private boolean positionWrapping = false;
        private double moi = 0.002;
        private PIDConstants pid = new PIDConstants(0.001, 0, 0);
        private FFConstants feedForward = FFConstants.INVALID;

        private Builder(final ControllerKind kind, final ControlMode mode, final DCMotor gearbox) {
            this.kind = kind;
            this.mode = mode;
            this.gearbox = gearbox;

            this.positionWrapping = mode == ControlMode.POSITION;
            this.brakeOnIdle = mode == ControlMode.POSITION;
        }

        public Builder sampleFrequency(double sampleFrequency) {
            this.sampleFrequency = sampleFrequency;
            return this;
        }

        public Builder inverted(boolean inverted) {
            this.inverted = inverted;
            return this;
        }

        public Builder encoderInverted(boolean encoderInverted) {
            this.encoderInverted = encoderInverted;
            return this;
        }

        public Builder gearing(double gearing) {
            this.gearing = gearing;
            return this;
        }

        public Builder encoderGearing(@Nullable Double encoderGearing) {
            this.encoderGearing = encoderGearing;
            return this;
        }

        public Builder currentLimit(int currentLimit) {
            this.currentLimit = currentLimit;
            return this;
        }

        public Builder brakeOnIdle(boolean brakeOnIdle) {
            this.brakeOnIdle = brakeOnIdle;
            return this;
        }

        public Builder positionWrapping(boolean positionWrapping) {
            this.positionWrapping = positionWrapping;
            return this;
        }

        public Builder moi(double moi) {
            this.moi = moi;
            return this;
        }

        public Builder pid(PIDConstants pid) {
            this.pid = pid;
            return this;
        }

        public Builder feedForward(FFConstants feedForward) {
            this.feedForward = feedForward;
            return this;
        }

        public MotorConfig build() {
            return new MotorConfig(this);
        }
    }
}
