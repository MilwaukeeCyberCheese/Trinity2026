package edu.msoe.cybercheese.trinity.subsystems.intake;

import edu.msoe.cybercheese.trinity.util.FFConstants;
import edu.msoe.cybercheese.trinity.util.PIDConstants;
import edu.msoe.cybercheese.trinity.util.hw.MotorConfig;
import edu.wpi.first.math.system.plant.DCMotor;

public class IntakeConstants {

    public static final int ROLLER_MOTOR_ID = 33;
    public static final MotorConfig ROLLER_MOTOR_CONFIG = new MotorConfig(
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

    public static final int LOWER_MOTOR_ID = 32;
    public static final MotorConfig LOWER_MOTOR_CONFIG = new MotorConfig(
            MotorConfig.ControllerKind.MAX,
            MotorConfig.ControlMode.POSITION,
            MotorConfig.DEFAULT_SAMPLE_FREQUENCY,
            false,
            false,
            DCMotor.getNeoVortex(1),
            27.,
            MotorConfig.VORTEX_CURRENT_LIMIT,
            true,
            true,
            0.002,
            new PIDConstants(2, 0, 0),
            new FFConstants(0.15, 0.019));
}
