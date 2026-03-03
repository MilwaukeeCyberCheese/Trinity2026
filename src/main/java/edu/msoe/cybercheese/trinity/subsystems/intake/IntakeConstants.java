package edu.msoe.cybercheese.trinity.subsystems.intake;

import edu.msoe.cybercheese.trinity.util.PIDConstants;
import edu.msoe.cybercheese.trinity.util.hw.MotorConfig;

public class IntakeConstants {

    public static final int ROLLER_MOTOR_ID = 33;
    public static final MotorConfig ROLLER_MOTOR_CONFIG = MotorConfig.vortexBuilder(
                    MotorConfig.ControllerKind.MAX, MotorConfig.ControlMode.VELOCITY)
            .gearing(21. / 50.)
            .build();

    public static final int LOWER_MOTOR_ID = 32;
    public static final MotorConfig LOWER_MOTOR_CONFIG = MotorConfig.vortexBuilder(
                    MotorConfig.ControllerKind.MAX, MotorConfig.ControlMode.POSITION)
            .gearing(27. * (50. / 18.))
            .pid(new PIDConstants(2, 0, 0))
            .build();
}
