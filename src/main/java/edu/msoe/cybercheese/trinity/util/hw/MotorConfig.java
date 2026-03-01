package edu.msoe.cybercheese.trinity.util.hw;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static edu.msoe.cybercheese.trinity.subsystems.drive.DriveConstants.*;

public record MotorConfig(
        ControllerKind kind,
        ControlMode mode,
        int canId,
        DCMotor gearbox,
        double gearing,
        double moi,
        int currentLimit,
        boolean brakeOnIdle,
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

    private static final double POSITION_FACTOR = 2. * Math.PI;
    private static final double VELOCITY_FACTOR = 2. * Math.PI / 60.;

    public SparkBaseConfig sparkConfig() {
        final var config = this.kind == ControllerKind.FLEX ? new SparkFlexConfig() : new SparkMaxConfig();

        // TODO: inversion?
        config
                .idleMode(this.brakeOnIdle ? SparkBaseConfig.IdleMode.kBrake : SparkBaseConfig.IdleMode.kCoast) // TODO
                .smartCurrentLimit(this.currentLimit)
                .voltageCompensation(12.0);

        config
                .encoder
                .positionConversionFactor(POSITION_FACTOR)
                .velocityConversionFactor(VELOCITY_FACTOR)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        config
                .absoluteEncoder
                .positionConversionFactor(POSITION_FACTOR)
                .velocityConversionFactor(VELOCITY_FACTOR)
                .averageDepth(2);

        // TODO: wrapping (just bool, always 0 to 2pi)
        config
                .closedLoop
                .feedbackSensor(
                        this.mode == ControlMode.POSITION
                                ? FeedbackSensor.kAbsoluteEncoder
                                : FeedbackSensor.kPrimaryEncoder)

                .pid(this.p, 0.0, this.d);

        // TODO: config frequency
        config
                .signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY))
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20);

        config
                .signals
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY))
                .absoluteEncoderVelocityAlwaysOn(true)
                .absoluteEncoderVelocityPeriodMs(20);

        config
                .signals
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);

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
