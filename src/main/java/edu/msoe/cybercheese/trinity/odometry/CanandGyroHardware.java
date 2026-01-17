package edu.msoe.cybercheese.trinity.odometry;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;

public class CanandGyroHardware implements GyroHardware {

    private final Canandgyro inner;

    public CanandGyroHardware(Canandgyro inner) {
        this.inner = inner;
    }

    @Override
    public double readYaw() {
        return this.inner.getMultiturnYaw();
    }
}
