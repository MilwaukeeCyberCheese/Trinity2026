package edu.msoe.cybercheese.trinity.subsystems.drive.gyro;

import edu.msoe.cybercheese.trinity.util.SparkUtil;
import edu.msoe.cybercheese.trinity.util.UnitTypes;
import edu.wpi.first.math.geometry.Rotation2d;
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
                gyroSimulation.getMeasuredAngularVelocity().in(UnitTypes.RADIANS_PER_SECOND);

        inputs.odometryYawTimestamps = SparkUtil.getSimulationOdometryTimeStamps(); // TODO: wtf
        inputs.odometryYawPositions = Arrays.stream(gyroSimulation.getCachedGyroReadings())
                .mapToDouble(Rotation2d::getRadians)
                .toArray();
    }
}
