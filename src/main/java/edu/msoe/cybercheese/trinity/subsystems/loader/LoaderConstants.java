package edu.msoe.cybercheese.trinity.subsystems.loader;

import edu.msoe.cybercheese.trinity.util.hw.MotorConfig;
import edu.wpi.first.math.util.Units;

public class LoaderConstants {

    public static final double WHEEL_RADIUS = Units.inchesToMeters(4);

    public static final int LOADER_MOTOR_ID = 36;
    public static final MotorConfig LOADER_MOTOR_CONFIG = MotorConfig.vortexBuilder(
                    MotorConfig.ControllerKind.MAX, MotorConfig.ControlMode.VELOCITY)
            .build();
}
