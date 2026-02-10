package edu.msoe.cybercheese.trinity.subsystems.drive.gyro;

import edu.msoe.cybercheese.trinity.util.SparkUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import java.util.Arrays;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroIOSim implements GyroIO {

    private final GyroSimulation gyroSimulation;

    public GyroIOSim(GyroSimulation gyroSimulation) {
        this.gyroSimulation = gyroSimulation;
    }

    @Override
    public void updateInputs(GyroInputs inputs) {
        inputs.connected = true;
        inputs.yawPosition = this.gyroSimulation.getGyroReading().getRadians();
        inputs.yawVelocityRadPerSec =
                gyroSimulation.getMeasuredAngularVelocity().in(Units.RadiansPerSecond);

        inputs.odometryYawTimestamps = SparkUtil.getSimulationOdometryTimeStamps(); // TODO: wtf
        inputs.odometryYawPositions = Arrays.stream(gyroSimulation.getCachedGyroReadings())
                .mapToDouble(Rotation2d::getRadians)
                .toArray();
    }
}
