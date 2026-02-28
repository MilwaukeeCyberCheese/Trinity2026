package edu.msoe.cybercheese.trinity.util.hw;

import static edu.msoe.cybercheese.trinity.subsystems.drive.DriveConstants.*;
import static edu.msoe.cybercheese.trinity.subsystems.drive.DriveConstants.ODOMETRY_FREQUENCY;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public record MotorConfig(
        boolean isFlex,
        ControlMode mode,
        int canId,
        DCMotor gearbox,
        double gearing,
        double moi,
        int currentLimit,
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

    public SparkBaseConfig sparkConfig() {
        final var config = this.isFlex ? new SparkFlexConfig() : new SparkMaxConfig();

        var driveConfig = new SparkFlexConfig();
        driveConfig
                .idleMode(SparkBaseConfig.IdleMode.kBrake) // TODO
                .smartCurrentLimit(this.currentLimit)
                .voltageCompensation(12.0);
        driveConfig
                .encoder
                .positionConversionFactor(this.positionFactor)
                .velocityConversionFactor(this.velocityFactor)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);
        driveConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(this.p, 0.0, this.d);
        driveConfig
                .signals
                .primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderPositionPeriodMs((int) (1000.0 / ODOMETRY_FREQUENCY))
                .primaryEncoderVelocityAlwaysOn(true)
                .primaryEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);

        return config;
    }

    public DCMotorSim buildSim() {
        return new DCMotorSim(LinearSystemId.createDCMotorSystem(this.gearbox, this.moi, this.gearing), this.gearbox);
    }

    public enum ControlMode {
        POSITION,
        VELOCITY,
    }
}
