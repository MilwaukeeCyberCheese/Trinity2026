package edu.msoe.cybercheese.trinity.odometry;

import edu.wpi.first.wpilibj.RobotController;
import it.unimi.dsi.fastutil.booleans.BooleanArrayList;
import it.unimi.dsi.fastutil.booleans.BooleanList;
import it.unimi.dsi.fastutil.doubles.DoubleArrayList;
import it.unimi.dsi.fastutil.doubles.DoubleList;
import java.util.concurrent.locks.ReentrantLock;

public final class NewOdometryCollectorThread {

  private final GyroHardware gyro;
  private final SwerveModuleHardware[] swerveModules;

  private final ReentrantLock bufferLock = new ReentrantLock();
  private OdometryBuffer currentBuffer = new OdometryBuffer();
  private OdometryBuffer mainThreadBuffer = new OdometryBuffer();

  private final boolean[] tempSwerveConnections;
  private final double[] tempDrivePositions;
  private final double[] tempTurnPositions;

  public NewOdometryCollectorThread(GyroHardware gyro, SwerveModuleHardware[] swerveModules) {
    this.gyro = gyro;
    this.swerveModules = swerveModules;
    this.tempSwerveConnections = new boolean[this.swerveModules.length];
    this.tempDrivePositions = new double[this.swerveModules.length];
    this.tempTurnPositions = new double[this.swerveModules.length];
  }

  public OdometryBuffer getMainThreadBuffer() {
    return mainThreadBuffer;
  }

  public void swapBuffers() {
    this.bufferLock.lock();
    try {
      final var newBuffer = this.mainThreadBuffer;
      this.mainThreadBuffer = this.currentBuffer;
      this.currentBuffer = newBuffer;
      this.currentBuffer.clear();
    } finally {
      this.bufferLock.unlock();
    }
  }

  public static final class OdometryBuffer {

    private final DoubleList timestamps = new DoubleArrayList();

    private boolean gyroConnected = false;
    private final DoubleList gyroRotations = new DoubleArrayList();

    private final BooleanList moduleConnections = new BooleanArrayList();
    private final DoubleList drivePositions = new DoubleArrayList();
    private final DoubleList turnPositions = new DoubleArrayList();

    public void clear() {
      this.timestamps.clear();
      this.gyroRotations.clear();
      this.drivePositions.clear();
      this.turnPositions.clear();
    }
  }

  private void readData() {
    final var timestamp = RobotController.getFPGATime();

    final var gyroConnected = false;
    final var gyroYaw = 0.0;

    for (int i = 0; i < this.swerveModules.length; i++) {
      this.tempSwerveConnections[i] = this.swerveModules[i].isConnected();
      this.tempDrivePositions[i] = this.swerveModules[i].readDrivePosition();
      this.tempTurnPositions[i] = this.swerveModules[i].readTurnPosition();
    }

    this.bufferLock.lock();
    try {
      this.currentBuffer.timestamps.add(timestamp);

      this.currentBuffer.gyroConnected = gyroConnected;
      this.currentBuffer.gyroRotations.add(gyroYaw);

      this.currentBuffer.moduleConnections.clear();

      for (int i = 0; i < this.swerveModules.length; i++) {
        this.currentBuffer.moduleConnections.add(this.tempSwerveConnections[i]);
        this.currentBuffer.drivePositions.add(this.tempDrivePositions[i]);
        this.currentBuffer.drivePositions.add(this.tempTurnPositions[i]);
      }
    } finally {
      this.bufferLock.unlock();
    }
  }
}
