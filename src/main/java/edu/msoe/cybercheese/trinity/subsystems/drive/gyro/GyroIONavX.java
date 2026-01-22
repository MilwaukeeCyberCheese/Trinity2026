package edu.msoe.cybercheese.trinity.subsystems.drive.gyro;

// import com.studica.frc.AHRS;
// import com.studica.frc.AHRS.NavXComType;

/** IO implementation for NavX. */
public class GyroIONavX implements GyroIO {
  //  private final AHRS navX = new AHRS(NavXComType.kMXP_SPI, (byte) odometryFrequency);
  //  private final Queue<Double> yawPositionQueue;
  //  private final Queue<Double> yawTimestampQueue;

  public GyroIONavX() {
    //    yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
    //    yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(navX::getAngle);
  }

  @Override
  public void updateInputs(GyroInputs inputs) {
    //    inputs.connected = navX.isConnected();
    //    inputs.yawPosition = Rotation2d.fromDegrees(-navX.getAngle());
    //    inputs.yawVelocityRadPerSec = Units.degreesToRadians(-navX.getRawGyroZ());
    //
    //    inputs.odometryYawTimestamps =
    //        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    //    inputs.odometryYawPositions =
    //        yawPositionQueue.stream()
    //            .map((Double value) -> Rotation2d.fromDegrees(-value))
    //            .toArray(Rotation2d[]::new);
    //    yawTimestampQueue.clear();
    //    yawPositionQueue.clear();
  }
}
