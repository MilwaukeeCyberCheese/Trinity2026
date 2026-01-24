package edu.msoe.cybercheese.trinity.subsystems.drive.gyro;

import edu.msoe.cybercheese.trinity.replay.IO;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface GyroIO extends IO<GyroIO.GyroInputs> {

    class GyroInputs implements LoggableInputs {
        public boolean connected = false;
        public double yawPosition = 0;
        public double yawVelocityRadPerSec = 0.0;
        public double[] odometryYawTimestamps = new double[] {};
        public double[] odometryYawPositions = new double[] {};

        @Override
        public void toLog(LogTable table) {
            table.put("connected", this.connected);
            table.put("yawPosition", this.yawPosition);
            table.put("yawVelocityRadPerSec", this.yawVelocityRadPerSec);
            table.put("odometryYawTimestamps", this.odometryYawTimestamps);
            table.put("odometryYawPositions", this.odometryYawPositions);
        }

        @Override
        public void fromLog(LogTable table) {
            this.connected = table.get("connected", this.connected);
            this.yawPosition = table.get("yawPosition", this.yawPosition);
            this.yawVelocityRadPerSec = table.get("yawVelocityRadPerSec", this.yawVelocityRadPerSec);
            this.odometryYawTimestamps = table.get("odometryYawTimestamps", this.odometryYawTimestamps);
            this.odometryYawPositions = table.get("odometryYawPositions", this.odometryYawPositions);
        }
    }
}
