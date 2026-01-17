package edu.msoe.cybercheese.trinity.odometry;

import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import edu.msoe.cybercheese.trinity.subsystems.drive.DriveConstants;

public class CanandgyroHardware implements GyroHardware {

    private Canandgyro inner = new Canandgyro(DriveConstants.CANANDGYRO_CAN_ID);

    @Override
    public boolean isConnected() {
        return this.inner.isConnected();
    }

    @Override
    public double readYaw() {
        return this.inner.getMultiturnYaw();
    }
}

