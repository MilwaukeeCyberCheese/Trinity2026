package edu.msoe.cybercheese.trinity.subsystems.drive;

import com.ctre.phoenix6.CANBus;
import edu.msoe.cybercheese.trinity.util.FFConstants;
import edu.msoe.cybercheese.trinity.util.PIDConstants;
import edu.msoe.cybercheese.trinity.util.UnitTypes;
import edu.msoe.cybercheese.trinity.util.hw.MotorConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

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
        new ModuleDefinition(16, 17, Rotation2d.fromDegrees(270)),
        // fr
        new ModuleDefinition(22, 23, Rotation2d.fromDegrees(0)), // TODO: someone put fr and br on backwards, fix on robot
        // bl
        new ModuleDefinition(20, 21, Rotation2d.fromDegrees(180)),
        // br
        new ModuleDefinition(18, 19, Rotation2d.fromDegrees(270)),
    };

    public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(1.5);

    public static final double TURN_MOTOR_TEMP_SIM_REDUCTION = 9424.0 / 203.0;

    public static final double ROBOT_MASS_KG = 74.088;
    public static final double ROBOT_MOI = 6.883;
    public static final double WHEEL_COF = 1.2;

    public static final int CANANDGYRO_CAN_ID = 61;
    public static final int PIGEON_CAN_ID = 60;
    public static final CANBus GYRO_CAN_BUS = CANBus.roboRIO();

    public static final double JOYSTICK_MULTIPLIER = 1.0;
    public static final double SLOW_MULTIPLIER = 0.5;
    public static final double CONTROLLER_INPUT_DEADBAND = 0.1;

    public static final MotorConfig DRIVE_MOTOR_CONFIG = MotorConfig.vortexBuilder(
                    MotorConfig.ControllerKind.FLEX, MotorConfig.ControlMode.VELOCITY)
            .sampleFrequency(ODOMETRY_FREQUENCY)
            .brakeOnIdle(true)
            .gearing((45.0 * 22.0) / (14.0 * 15.0))
            .pid(new PIDConstants(0.0105, 0, 0.0001))
            .feedForward(new FFConstants(0, 0.25))
            .currentLimit(35)
            .build();

    public static final MotorConfig TURN_MOTOR_CONFIG = MotorConfig.builder(
                    MotorConfig.ControllerKind.MAX, MotorConfig.ControlMode.POSITION, DCMotor.getNeo550(1))
            .sampleFrequency(ODOMETRY_FREQUENCY)
            .positionWrapping(true)
            .encoderInverted(true)
            .currentLimit(20)
            .pid(new PIDConstants(2, 0, 0))
            .build();

    public static final SwerveModuleSimulationConfig SWERVE_MODULE_SIMULATION_CONFIG = new SwerveModuleSimulationConfig(
            DRIVE_MOTOR_CONFIG.gearbox(),
            TURN_MOTOR_CONFIG.gearbox(),
            DRIVE_MOTOR_CONFIG.gearing(),
            TURN_MOTOR_TEMP_SIM_REDUCTION,
            UnitTypes.VOLTS.of(0.1),
            UnitTypes.VOLTS.of(0.1),
            UnitTypes.METERS.of(WHEEL_RADIUS_METERS),
            UnitTypes.KILOGRAM_SQUARE_METERS.of(0.02),
            WHEEL_COF);

    public static final DriveTrainSimulationConfig DRIVE_TRAIN_SIMULATION_CONFIG = new DriveTrainSimulationConfig(
            UnitTypes.KILOGRAMS.of(ROBOT_MASS_KG),
            UnitTypes.METERS.of(WHEEL_BASE + 0.1),
            UnitTypes.METERS.of(TRACK_WIDTH + 0.1),
            UnitTypes.METERS.of(WHEEL_BASE),
            UnitTypes.METERS.of(WHEEL_BASE),
            () -> new GyroSimulation(0.5, 0.02),
            () -> new SwerveModuleSimulation(SWERVE_MODULE_SIMULATION_CONFIG));

    public record ModuleDefinition(int driveCanId, int turnCanId, Rotation2d zeroRotation) {}
}
