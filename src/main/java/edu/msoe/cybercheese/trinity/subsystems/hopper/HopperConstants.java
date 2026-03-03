package edu.msoe.cybercheese.trinity.subsystems.hopper;

import edu.msoe.cybercheese.trinity.util.FFConstants;
import edu.msoe.cybercheese.trinity.util.PIDConstants;
import edu.msoe.cybercheese.trinity.util.hw.MotorConfig;
import edu.wpi.first.math.system.plant.DCMotor;

public class HopperConstants {

    public static final int HOPPER_MOTOR_ID = 34;

    public static final MotorConfig HOPPER_MOTOR_CONFIG = new MotorConfig(
            MotorConfig.ControllerKind.MAX,
            MotorConfig.ControlMode.VELOCITY,
            MotorConfig.DEFAULT_SAMPLE_FREQUENCY,
            false,
            false,
            DCMotor.getNeoVortex(1),
            64.,
            50,
            false,
            false,
            0.002,
            new PIDConstants(0.001, 0, 0),
            new FFConstants(0.15, 0.019));
}
