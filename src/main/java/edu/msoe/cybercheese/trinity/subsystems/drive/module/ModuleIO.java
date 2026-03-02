package edu.msoe.cybercheese.trinity.subsystems.drive.module;

import edu.msoe.cybercheese.trinity.replay.IO;
import edu.msoe.cybercheese.trinity.util.hw.MotorIO;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ModuleIO extends IO<ModuleIO.ModuleInputs> {

    class ModuleInputs implements LoggableInputs {

        public MotorIO.MotorInputs drive = new MotorIO.MotorInputs();

        public boolean turnConnected = false;
        public double turnPosition = 0.0;
        public double turnVelocity = 0.0;
        public double turnAppliedVolts = 0.0;
        public double turnCurrentAmps = 0.0;

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositions = new double[] {};
        public double[] odometryTurnPositions = new double[] {};

        @Override
        public void toLog(LogTable table) {
            table.put("drive", this.drive);

            table.put("turnConnected", this.turnConnected);
            table.put("turnPosition", this.turnPosition);
            table.put("turnVelocity", this.turnVelocity);
            table.put("turnAppliedVolts", this.turnAppliedVolts);
            table.put("turnCurrentAmps", this.turnCurrentAmps);

            table.put("odometryTimestamps", this.odometryTimestamps);
            table.put("odometryDrivePositions", this.odometryDrivePositions);
            table.put("odometryTurnPositions", this.odometryTurnPositions);
        }

        @Override
        public void fromLog(LogTable table) {
            this.drive = table.get("drive", new MotorIO.MotorInputs());

            this.turnConnected = table.get("turnConnected", this.turnConnected);
            this.turnPosition = table.get("turnPosition", this.turnPosition);
            this.turnVelocity = table.get("turnVelocity", this.turnVelocity);
            this.turnAppliedVolts = table.get("turnAppliedVolts", this.turnAppliedVolts);
            this.turnCurrentAmps = table.get("turnCurrentAmps", this.turnCurrentAmps);

            this.odometryTimestamps = table.get("odometryTimestamps", this.odometryTimestamps);
            this.odometryDrivePositions = table.get("odometryDrivePositions", this.odometryDrivePositions);
            this.odometryTurnPositions = table.get("odometryTurnPositions", this.odometryTurnPositions);
        }
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
