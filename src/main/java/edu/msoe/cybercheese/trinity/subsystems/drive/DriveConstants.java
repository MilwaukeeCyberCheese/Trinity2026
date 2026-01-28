package edu.msoe.cybercheese.trinity.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DriveConstants {

    public static final double MAX_SPEED = 4.8;
    public static final double ODOMETRY_FREQUENCY = 250.0; // Hz
    public static final double TRACK_WIDTH = Units.inchesToMeters(28);
    public static final double WHEEL_BASE = Units.inchesToMeters(28);
    public static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0);


    public static final Translation2d[] MODULE_TRANSLATIONS = new Translation2d[] {
            new Translation2d(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0),
            new Translation2d(TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0),
            new Translation2d(-TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0),
            new Translation2d(-TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0)
    };

    public static final ModuleDefinition[] MODULE_DEFINITIONS = new ModuleDefinition[] {
            // fl
            new ModuleDefinition(1, 2, Rotation2d.fromDegrees(-90)),
            // fr
            new ModuleDefinition(11, 12, Rotation2d.fromDegrees(0)),
            // bl
            new ModuleDefinition(5, 6, Rotation2d.fromDegrees(180)),
            // br
            new ModuleDefinition(7, 8, Rotation2d.fromDegrees(90)),
    };

    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 50;
    public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(1.5);
    // MAXSwerve with 14 pinion teeth and 22 spur teeth
    public static final double DRIVE_MOTOR_REDUCTION = (45.0 * 22.0) / (14.0 * 15.0);
    public static final DCMotor DRIVE_GEARBOX = DCMotor.getNeoVortex(1);

    // Drive encoder configuration
    public static final double DRIVE_ENCODER_POSITION_FACTOR =
            2 * Math.PI / DRIVE_MOTOR_REDUCTION; // Rotor Rotations ->
    // Wheel Radians
    public static final double DRIVE_ENCODER_VELOCITY_FACTOR =
            (2 * Math.PI) / 60.0 / DRIVE_MOTOR_REDUCTION; // Rotor RPM ->
    // Wheel Rad/Sec

    public static final double DRIVE_KP = 0.0;
    public static final double DRIVE_KD = 0.0;
    public static final double DRIVE_KS = 0.0;
    public static final double DRIVE_KV = 0.1;
    public static final double DRIVE_SIM_P = 0.05;
    public static final double DRIVE_SIM_D = 0.0;
    public static final double DRIVE_SIM_KS = 0.0;
    public static final double DRIVE_SIM_KV = 0.0789;

    public static final boolean TURN_INVERTED = false;
    public static final int TURN_MOTOR_CURRENT_LIMIT = 20;
    public static final double TURN_MOTOR_REDUCTION = 9424.0 / 203.0;
    public static final DCMotor TURN_GEARBOX = DCMotor.getNeo550(1);

    public static final boolean TURN_ENCODER_INVERTED = true;
    // Rotations -> Radians
    public static final double TURN_ENCODER_POSITION_FACTOR = 2 * Math.PI;
    // RPM -> Rad/Sec
    public static final double TURN_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0;

    public static final double TURN_KP = 2.0;
    public static final double TURN_KD = 0.0;
    public static final double TURN_SIM_P = 8.0;
    public static final double TURN_SIM_D = 0.0;
    public static final double TURN_PID_MIN_INPUT = 0; // Radians
    public static final double TURN_PID_MAX_INPUT = 2 * Math.PI; // Radians

    public static final double ROBOT_MASS_KG = 74.088;
    public static final double ROBOT_MOI = 6.883;
    public static final double WHEEL_COF = 1.2;

    public static final int CANANDGYRO_CAN_ID = -1;

    public record ModuleDefinition(
            int driveCanId,
            int turnCanId,
            Rotation2d zeroRotation
    ) {}
}
