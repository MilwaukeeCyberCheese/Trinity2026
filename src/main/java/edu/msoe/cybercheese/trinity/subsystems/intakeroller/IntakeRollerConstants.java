package edu.msoe.cybercheese.trinity.subsystems.intakeroller;

import edu.msoe.cybercheese.trinity.util.hw.MotorConfig;

public class IntakeRollerConstants {

    public static final int ROLLER_MOTOR_ID = 33;
    public static final MotorConfig ROLLER_MOTOR_CONFIG = MotorConfig.vortexBuilder(
                    MotorConfig.ControllerKind.MAX, MotorConfig.ControlMode.VELOCITY)
            .gearing(21. / 50.)
            .currentLimit(30)
            .build();
}
