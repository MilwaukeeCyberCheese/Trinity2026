package edu.msoe.cybercheese.trinity.odometry;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import it.unimi.dsi.fastutil.doubles.DoubleArrayList;
import it.unimi.dsi.fastutil.doubles.DoubleList;

public class Pigeon2Hardware implements OdometryCallback {
    private static final double UPDATE_FREQUENCY_HZ = 250.0;
    private static final double UPDATE_TIMEOUT_SECONDS = 0.02;

    private final StatusSignal<Angle> yawSignal;

    public final DoubleList timestamps = new DoubleArrayList();
    public final DoubleList rotations = new DoubleArrayList();

    public Pigeon2Hardware(Pigeon2 pigeon) {
        this.yawSignal = pigeon.getYaw();
        this.yawSignal.setUpdateFrequency(UPDATE_FREQUENCY_HZ);

        pigeon.optimizeBusUtilization();
    }

    @Override
    public void clearFrame() {
        this.timestamps.clear();
        this.rotations.clear();
    }

    @Override
    public void collectOdometry(double fpgaTime) {
        boolean success = BaseStatusSignal.waitForAll(UPDATE_TIMEOUT_SECONDS, this.yawSignal)
                .isOK();

        if (success) {
            this.timestamps.add(fpgaTime);
            this.rotations.add(this.yawSignal.getValue().in(Units.Radians));
        }
    }
}
