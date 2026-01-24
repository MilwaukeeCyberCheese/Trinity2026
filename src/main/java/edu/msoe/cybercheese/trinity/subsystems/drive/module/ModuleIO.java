package edu.msoe.cybercheese.trinity.subsystems.drive.module;

import edu.msoe.cybercheese.trinity.replay.IO;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ModuleIO extends IO<ModuleIO.ModuleInputs> {

    class ModuleInputs implements LoggableInputs {
        public boolean driveConnected = false;
        public double drivePosition = 0.0;
        public double driveVelocity = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public boolean turnConnected = false;
        public double turnPosition = 0.0;
        public double turnVelocity = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositions = new double[] {};
        public double[] odometryTurnPositions = new double[] {};
    }

    /** Run the drive motor at the specified open loop value. */
    default void setDriveOpenLoop(double output) {}

    /** Run the turn motor at the specified open loop value. */
    default void setTurnOpenLoop(double output) {}

    /** Run the drive motor at the specified velocity. */
    default void setDriveVelocity(double velocityRadPerSec) {}

    /** Run the turn motor to the specified rotation. */
    default void setTurnPosition(Rotation2d rotation) {}
}
