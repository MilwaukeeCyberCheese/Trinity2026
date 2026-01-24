package edu.msoe.cybercheese.trinity.odometry;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import edu.wpi.first.math.util.Units;
import it.unimi.dsi.fastutil.doubles.DoubleArrayList;
import it.unimi.dsi.fastutil.doubles.DoubleList;

public class CanandGyroHardware implements OdometryCallback {

    private final Canandgyro inner;
    public final DoubleList timestamps = new DoubleArrayList();
    public final DoubleList rotations = new DoubleArrayList();

    public CanandGyroHardware(Canandgyro inner) {
        this.inner = inner;
    }

    @Override
    public void clearFrame() {
        this.timestamps.clear();
        this.rotations.clear();
    }

    @Override
    public void collectOdometry(double fpgaTime) {
        this.timestamps.add(fpgaTime);
        this.rotations.add(Units.rotationsToRadians(this.inner.getMultiturnYaw()));
    }
}
