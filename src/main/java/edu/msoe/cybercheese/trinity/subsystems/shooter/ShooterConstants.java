package edu.msoe.cybercheese.trinity.subsystems.shooter;

import edu.msoe.cybercheese.trinity.util.hw.MotorConfig;
import edu.wpi.first.math.util.Units;

public class ShooterConstants {

    public static final double WHEEL_RADIUS = Units.inchesToMeters(4);
    public static final double VELOCITY_ADJUSTMENT = 1.0;

    public static final int SHOOTER_MOTOR_ID = 35;
    public static final MotorConfig SHOOTER_MOTOR_CONFIG = MotorConfig.vortexBuilder(
                    MotorConfig.ControllerKind.MAX, MotorConfig.ControlMode.VELOCITY)
            .build();
}
