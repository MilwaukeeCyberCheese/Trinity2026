package edu.msoe.cybercheese.trinity.replay;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public final class LogUtil {

    public static <T extends LoggableInputs> void update(final String key, final IO<T> io, final T inputs) {
        io.updateInputs(inputs);

        Logger.processInputs(key, inputs);
    }

    private LogUtil() {}
}
