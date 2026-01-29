package edu.msoe.cybercheese.trinity.subsystems.drive.gyro;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import edu.msoe.cybercheese.trinity.odometry.CanandGyroHardware;
import edu.msoe.cybercheese.trinity.odometry.OdometryCallback;
import edu.msoe.cybercheese.trinity.subsystems.drive.DriveConstants;
import edu.wpi.first.math.util.Units;
import org.jspecify.annotations.Nullable;

public class GyroIOCanandGyro implements GyroIO {

    private final Canandgyro inner = new Canandgyro(DriveConstants.CANANDGYRO_CAN_ID);

    private final CanandGyroHardware gyroHal = new CanandGyroHardware(this.inner);

    @Override
    public void updateInputs(GyroInputs inputs) {
        inputs.connected = this.inner.isConnected();
        inputs.yawPosition = Units.rotationsToRadians(this.inner.getMultiturnYaw());
        inputs.yawVelocityRadPerSec = Units.rotationsToRadians(this.inner.getAngularVelocityYaw());

        inputs.odometryYawTimestamps = this.gyroHal.timestamps.toDoubleArray();
        inputs.odometryYawPositions = this.gyroHal.rotations.toDoubleArray();
    }

    @Override
    public @Nullable OdometryCallback getOdometryCallback() {
        return this.gyroHal;
    }
}
