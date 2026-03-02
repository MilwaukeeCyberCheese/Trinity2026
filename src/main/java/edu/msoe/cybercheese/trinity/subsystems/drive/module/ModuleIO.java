package edu.msoe.cybercheese.trinity.subsystems.drive.module;

import edu.msoe.cybercheese.trinity.replay.IO;
import edu.msoe.cybercheese.trinity.util.hw.MotorIO;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ModuleIO extends IO<ModuleIO.ModuleInputs> {

    class ModuleInputs implements LoggableInputs {

        public MotorIO.MotorInputs drive = new MotorIO.MotorInputs();
        public MotorIO.MotorInputs turn = new MotorIO.MotorInputs();

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositions = new double[] {};
        public double[] odometryTurnPositions = new double[] {};

        @Override
        public void toLog(LogTable table) {
            table.put("drive", this.drive);
            table.put("turn", this.turn);

            table.put("odometryTimestamps", this.odometryTimestamps);
            table.put("odometryDrivePositions", this.odometryDrivePositions);
            table.put("odometryTurnPositions", this.odometryTurnPositions);
        }

        @Override
        public void fromLog(LogTable table) {
            this.drive = table.get("drive", new MotorIO.MotorInputs());
            this.turn = table.get("turn", new MotorIO.MotorInputs());

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
