package edu.msoe.cybercheese.trinity.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class ShooterConstants {
    public static final int SHOOTER_MOTOR_ID = 35;

    public static final DCMotor SHOOTER_GEARBOX = DCMotor.getNeoVortex(1);
    public static final double SHOOTER_GEARING = 1.0;

    public static final double SHOOTER_MOI = 0.002;

    public static final int SHOOTER_CURRENT_LIMIT = 50;
    public static final double SHOOTER_VELOCITY_FACTOR = Units.rotationsPerMinuteToRadiansPerSecond(1.0);

    public static final double SHOOTER_KS = 0.15;
    public static final double SHOOTER_KV = 0.019;
    public static final double SHOOTER_KP = 0.001;
    public static final double SHOOTER_KD = 0.0;

    public static final double SHOOTER_SIM_P = 0.5;
    public static final double SHOOTER_SIM_D = 0.0;
    public static final double SHOOTER_SIM_KS = 0.0;
    public static final double SHOOTER_SIM_KV = 0.0175;

    public static final double WHEEL_RADIUS = Units.inchesToMeters(4);
    public static final double VELOCITY_CONVERSION_FACTOR = 2 * Math.PI / WHEEL_RADIUS;
}
