package edu.msoe.cybercheese.trinity.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class IntakeConstants {

    public static final int ROLLER_MOTOR_ID = 34;

    public static final DCMotor ROLLER_GEARBOX = DCMotor.getNeoVortex(1);
    public static final double ROLLER_GEARING = 1.0;

    public static final double ROLLER_MOI = 0.002;

    public static final int ROLLER_CURRENT_LIMIT = 50;
    public static final double ROLLER_VELOCITY_FACTOR = Units.rotationsPerMinuteToRadiansPerSecond(1.0);

    public static final double ROLLER_KS = 0.15;
    public static final double ROLLER_KV = 0.019;
    public static final double ROLLER_KP = 0.001;
    public static final double ROLLER_KD = 0.0;

    public static final double ROLLER_SIM_P = 0.5;
    public static final double ROLLER_SIM_D = 0.0;
    public static final double ROLLER_SIM_KS = 0.0;
    public static final double ROLLER_SIM_KV = 0.0175;

    public static final int LOWER_MOTOR_ID = 32;

    public static final int LOWER_MOTOR_CURRENT_LIMIT = 50;
    public static final double LOWER_MOTOR_REDUCTION = 1.0;
    public static final DCMotor LOWER_GEARBOX = DCMotor.getNeoVortex(1);
    // Rotations -> Radians
    public static final double LOWER_ENCODER_POSITION_FACTOR = 2 * Math.PI;
    // RPM -> Rad/Sec
    public static final double LOWER_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0;

    public static final double LOWER_KP = 2.0;
    public static final double LOWER_KD = 0.0;
    public static final double LOWER_SIM_P = 8.0;
    public static final double LOWER_SIM_D = 0.0;
    public static final double LOWER_PID_MIN_INPUT = 0; // Radians
    public static final double LOWER_PID_MAX_INPUT = 2 * Math.PI; // Radians
}
