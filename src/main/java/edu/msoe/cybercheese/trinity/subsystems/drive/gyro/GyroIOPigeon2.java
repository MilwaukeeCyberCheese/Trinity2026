package edu.msoe.cybercheese.trinity.subsystems.drive.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.msoe.cybercheese.trinity.odometry.OdometryCallback;
import edu.msoe.cybercheese.trinity.odometry.Pigeon2Hardware;
import edu.msoe.cybercheese.trinity.subsystems.drive.DriveConstants;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.jspecify.annotations.Nullable;

public class GyroIOPigeon2 implements GyroIO {
    private static final double SIGNAL_UPDATE_FREQUENCY_HZ = 100.0;

    private final Pigeon2 pigeon;
    private final StatusSignal<Angle> yaw;
    private final StatusSignal<AngularVelocity> yawVelocity;
    private final Pigeon2Hardware gyroHal;

    public GyroIOPigeon2() {
        pigeon = new Pigeon2(DriveConstants.PIGEON_CAN_ID, DriveConstants.GYRO_CAN_BUS);

        Pigeon2Configuration config = new Pigeon2Configuration();
        config.Pigeon2Features.EnableCompass = false;
        pigeon.getConfigurator().apply(config);

        yaw = pigeon.getYaw();
        yawVelocity = pigeon.getAngularVelocityZWorld();
        yaw.setUpdateFrequency(SIGNAL_UPDATE_FREQUENCY_HZ);
        yawVelocity.setUpdateFrequency(SIGNAL_UPDATE_FREQUENCY_HZ);
        pigeon.optimizeBusUtilization();
        gyroHal = new Pigeon2Hardware(pigeon);
    }

    @Override
    public void updateInputs(GyroInputs inputs) {
        BaseStatusSignal.refreshAll(yaw, yawVelocity);

        inputs.connected = BaseStatusSignal.isAllGood(yaw, yawVelocity);
        inputs.yawPosition = yaw.getValue().in(Units.Radians);
        inputs.yawVelocityRadPerSec = yawVelocity.getValue().in(Units.RadiansPerSecond);
        inputs.odometryYawTimestamps = gyroHal.timestamps.toDoubleArray();
        inputs.odometryYawPositions = gyroHal.rotations.toDoubleArray();

        gyroHal.clearFrame();
    }

    @Override
    public @Nullable OdometryCallback getOdometryCallback() {
        return gyroHal;
    }
}
