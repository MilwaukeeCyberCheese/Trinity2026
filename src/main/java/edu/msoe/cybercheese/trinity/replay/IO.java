package edu.msoe.cybercheese.trinity.replay;

import edu.msoe.cybercheese.trinity.odometry.OdometryCallback;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface IO<T extends LoggableInputs> {

    void updateInputs(T inputs);

    default @Nullable OdometryCallback getOdometryCallback() {
        return null;
    }
}
