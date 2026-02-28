package edu.msoe.cybercheese.trinity.util.hw;

import edu.wpi.first.math.system.plant.DCMotor;

public record MotorConfig(
        int canId,
        DCMotor gearbox,
        double gearing,

        double moi,

        double currentLimit,

        double positionFactor,
        double velocityFactor,

        double kp
//        double

        ) {
}
