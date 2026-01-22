package edu.msoe.cybercheese.trinity.subsystems.drive.gyro;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import edu.msoe.cybercheese.trinity.subsystems.drive.DriveConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class GyroIOCanandGyro implements GyroIO {

  private final Canandgyro inner = new Canandgyro(DriveConstants.CANANDGYRO_CAN_ID);

  public GyroIOCanandGyro() {}

  private Rotation2d readAngle() {
    return Rotation2d.fromRotations(this.inner.getYaw());
  }

  @Override
  public void updateInputs(GyroInputs inputs) {
    inputs.connected = inner.isConnected();
    inputs.yawPosition = this.readAngle();
    inputs.yawVelocityRadPerSec = Units.rotationsToRadians(this.inner.getAngularVelocityYaw());

    inputs.odometryYawTimestamps = this.yawTimestampQueue.toArray(double[]::new);
    inputs.odometryYawPositions = this.yawPositionQueue.toArray(double[]::new);
  }
}
