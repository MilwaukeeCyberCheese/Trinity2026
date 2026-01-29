package edu.msoe.cybercheese.trinity.subsystems.drive.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.msoe.cybercheese.trinity.odometry.OdometryCallback;
import edu.msoe.cybercheese.trinity.odometry.Pigeon2Hardware;
import edu.msoe.cybercheese.trinity.subsystems.drive.DriveConstants;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.jspecify.annotations.Nullable;

public class GyroIOPigeon2 implements GyroIO {

    private final Pigeon2 pigeon;
    private final StatusSignal<Angle> yaw;
    private final StatusSignal<AngularVelocity> yawVelocity;
    private final Pigeon2Hardware gyroHal;

    public GyroIOPigeon2() {
        pigeon = new Pigeon2(DriveConstants.PIGEON_CAN_ID, DriveConstants.GYRO_CAN_BUS);
        pigeon.getConfigurator().apply(new Pigeon2Configuration());

        yaw = pigeon.getYaw();
        yawVelocity = pigeon.getAngularVelocityZWorld();
        gyroHal = new Pigeon2Hardware(pigeon);
    }

    @Override
    public void updateInputs(GyroInputs inputs) {
        BaseStatusSignal.refreshAll(yaw, yawVelocity);

        inputs.connected = BaseStatusSignal.isAllGood(yaw, yawVelocity);
        
        // rotations -> radians
        inputs.yawPosition = Units.rotationsToRadians(yaw.getValueAsDouble());
        // rotations/s -> radians/s
        inputs.yawVelocityRadPerSec = Units.rotationsToRadians(yawVelocity.getValueAsDouble());

        inputs.odometryYawTimestamps = gyroHal.timestamps.toDoubleArray();
        inputs.odometryYawPositions = gyroHal.rotations.toDoubleArray();
    }

    @Override
    public @Nullable OdometryCallback getOdometryCallback() {
        return gyroHal;
    }
}