package edu.msoe.cybercheese.trinity.subsystems.hopper;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class HopperConstants {

    public static final int HOPPER_MOTOR_ID = 34;

    public static final DCMotor HOPPER_GEARBOX = DCMotor.getNeoVortex(1);
    public static final double HOPPER_GEARING = 1.0;

    public static final double HOPPER_MOI = 0.002;

    public static final int HOPPER_CURRENT_LIMIT = 50;
    public static final double HOPPER_VELOCITY_FACTOR = Units.rotationsPerMinuteToRadiansPerSecond(1.0);

    public static final double HOPPER_KS = 0.15;
    public static final double HOPPER_KV = 0.019;
    public static final double HOPPER_KP = 0.001;
    public static final double HOPPER_KD = 0.0;

    public static final double HOPPER_SIM_P = 0.5;
    public static final double HOPPER_SIM_D = 0.0;
    public static final double HOPPER_SIM_KS = 0.0;
    public static final double HOPPER_SIM_KV = 0.0175;
}
