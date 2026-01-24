package edu.msoe.cybercheese.trinity.odometry;

import edu.msoe.cybercheese.trinity.subsystems.drive.DriveConstants;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import org.jspecify.annotations.Nullable;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.ReentrantLock;

public final class OdometryCollector implements AutoCloseable {

    private static final AtomicInteger INIT_COUNT = new AtomicInteger();

    private final ReentrantLock bufferLock = new ReentrantLock();
    private List<OdometryCallback> callbacks = new ArrayList<>();

    private final Notifier notifier = new Notifier(this::readData);

    public OdometryCollector() {
        this.notifier.setName(this.getClass().getSimpleName() + "-" + INIT_COUNT.getAndIncrement());
    }

    public void addCallback(final @Nullable OdometryCallback callback) {
        this.callbacks.add(callback);
    }

    public void start() {
        this.callbacks = Collections.unmodifiableList(this.callbacks);
        this.notifier.startPeriodic(1.0 / DriveConstants.ODOMETRY_FREQUENCY);
    }

    @Override
    public void close() {
        this.notifier.stop();
    }

    public void clearAll() {
        for (final var cb : this.callbacks) {
            cb.clearFrame();
        }
    }

    private void readData() {
        final var fpgaTime = RobotController.getFPGATime();

        this.bufferLock.lock();
        try {
            for (final var cb : this.callbacks) {
                cb.clearFrame();
                cb.collectOdometry(fpgaTime);
            }
        } finally {
            this.bufferLock.unlock();
        }
    }

}
