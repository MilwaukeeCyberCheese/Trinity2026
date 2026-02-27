package edu.msoe.cybercheese.trinity.subsystems.loader;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class LoaderConstants {

    public static final int LOADER_MOTOR_ID = 36;

    public static final DCMotor LOADER_GEARBOX = DCMotor.getNeoVortex(1);
    public static final double LOADER_GEARING = 1.0;

    public static final double LOADER_MOI = 0.002;

    public static final int LOADER_CURRENT_LIMIT = 50;
    public static final double LOADER_VELOCITY_FACTOR = Units.rotationsPerMinuteToRadiansPerSecond(1.0);

    public static final double LOADER_KS = 0.15;
    public static final double LOADER_KV = 0.019;
    public static final double LOADER_KP = 0.001;
    public static final double LOADER_KD = 0.0;

    public static final double LOADER_SIM_P = 0.5;
    public static final double LOADER_SIM_D = 0.0;
    public static final double LOADER_SIM_KS = 0.0;
    public static final double LOADER_SIM_KV = 0.0175;

    public static final double WHEEL_RADIUS = Units.inchesToMeters(4);
    public static final double VELOCITY_CONVERSION_FACTOR = 2 * Math.PI / WHEEL_RADIUS;
}
