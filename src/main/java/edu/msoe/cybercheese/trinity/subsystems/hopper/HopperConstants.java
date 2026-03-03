package edu.msoe.cybercheese.trinity.subsystems.hopper;

import edu.msoe.cybercheese.trinity.util.hw.MotorConfig;

public class HopperConstants {

    public static final int HOPPER_MOTOR_ID = 34;
    public static final MotorConfig HOPPER_MOTOR_CONFIG = MotorConfig.vortexBuilder(
                    MotorConfig.ControllerKind.MAX, MotorConfig.ControlMode.VELOCITY)
            .gearing(64)
            .build();
}
