package edu.msoe.cybercheese.trinity.odometry;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import it.unimi.dsi.fastutil.doubles.DoubleArrayList;
import it.unimi.dsi.fastutil.doubles.DoubleList;

public class Pigeon2Hardware implements OdometryCallback {

    private final StatusSignal<Angle> yawSignal;
    public final DoubleList timestamps = new DoubleArrayList();
    public final DoubleList rotations = new DoubleArrayList();

    public Pigeon2Hardware(Pigeon2 pigeon) {
        this.yawSignal = pigeon.getYaw();
        this.yawSignal.setUpdateFrequency(250.0);
    }

    @Override
    public void clearFrame() {
        this.timestamps.clear();
        this.rotations.clear();
    }

    @Override
    public void collectOdometry(double fpgaTime) {
        this.yawSignal.waitForUpdate(0.02); // wait up to 20ms or give up
        this.timestamps.add(fpgaTime);
        this.rotations.add(Units.rotationsToRadians(this.yawSignal.getValueAsDouble()));
    }
}
