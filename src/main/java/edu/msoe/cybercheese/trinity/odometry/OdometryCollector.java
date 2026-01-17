package edu.msoe.cybercheese.trinity.odometry;

import edu.wpi.first.wpilibj.RobotController;
import it.unimi.dsi.fastutil.doubles.DoubleArrayList;
import it.unimi.dsi.fastutil.doubles.DoubleList;

import java.util.concurrent.locks.ReentrantLock;

public final class OdometryCollector {
    
    private final GyroHardware gyro;
    private final SwerveModuleHardware[] swerveModules;
    
    private final ReentrantLock bufferLock = new ReentrantLock();
    private OdometryBuffer currentBuffer = new OdometryBuffer();
    private OdometryBuffer mainThreadBuffer = new OdometryBuffer();
    
    public OdometryCollector(GyroHardware gyro, SwerveModuleHardware[] swerveModules) {
        this.gyro = gyro;
        this.swerveModules = swerveModules;
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

        public final DoubleList timestamps = new DoubleArrayList();

        public final DoubleList gyroRotations = new DoubleArrayList();

        public final DoubleList drivePositions = new DoubleArrayList();
        public final DoubleList turnPositions = new DoubleArrayList();

        public void clear() {
            this.timestamps.clear();
            this.gyroRotations.clear();
            this.drivePositions.clear();
            this.turnPositions.clear();
        }
    }

    private void readData() {
        this.bufferLock.lock();
        try {
            this.currentBuffer.timestamps.add(RobotController.getFPGATime());

            this.currentBuffer.gyroRotations.add(this.gyro.readYaw());

            for (final var swerveModule : this.swerveModules) {
                this.currentBuffer.drivePositions.add(swerveModule.readDrivePosition());
                this.currentBuffer.turnPositions.add(swerveModule.readTurnPosition());
            }
        } finally {
            this.bufferLock.unlock();
        }
    }

}
