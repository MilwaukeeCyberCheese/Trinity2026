package edu.msoe.cybercheese.trinity.subsystems.loader;

import edu.msoe.cybercheese.trinity.util.FFConstants;
import edu.msoe.cybercheese.trinity.util.PIDConstants;
import edu.msoe.cybercheese.trinity.util.hw.MotorConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class LoaderConstants {

    public static final double WHEEL_RADIUS = Units.inchesToMeters(4);

    public static final int LOADER_MOTOR_ID = 36;
    public static final MotorConfig LOADER_MOTOR_CONFIG = new MotorConfig(
            MotorConfig.ControllerKind.MAX,
            MotorConfig.ControlMode.VELOCITY,
            MotorConfig.DEFAULT_SAMPLE_FREQUENCY,
            false,
            false,
            DCMotor.getNeoVortex(1),
            1.,
            MotorConfig.VORTEX_CURRENT_LIMIT,
            false,
            false,
            0.002,
            new PIDConstants(0.001, 0, 0),
            new FFConstants(0.15, 0.019));
}
