package edu.msoe.cybercheese.trinity.subsystems.drive.gyro;

import edu.msoe.cybercheese.trinity.replay.IO;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface GyroIO extends IO<GyroIO.GyroInputs> {

    class GyroInputs implements LoggableInputs {
        public boolean connected = false;
        public Rotation2d yawPosition = Rotation2d.kZero;
        public double yawVelocityRadPerSec = 0.0;
        public double[] odometryYawTimestamps = new double[] {};
        public double[] odometryYawPositions = new double[] {};
    }
}
