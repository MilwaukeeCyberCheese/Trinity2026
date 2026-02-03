package edu.msoe.cybercheese.trinity.subsystems.hopper;

import edu.wpi.first.math.system.plant.DCMotor;

public class HopperConstants {
    public static final double GEAR_RATIO = 1.0;
    public static final double MOI_KG_M2 = 0.001;
    public static final double FORWARD_VOLTAGE = 6.0;
    public static final int HOPPER_MOTOR_ID = 10; // TODO: Set correct ID
    public static final DCMotor HOPPER_GEARBOX = DCMotor.getNeoVortex(1);

}