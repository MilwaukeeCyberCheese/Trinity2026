package edu.msoe.cybercheese.trinity;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import com.reduxrobotics.canand.CanandEventLoop;
import edu.msoe.cybercheese.trinity.commands.DriveCommands;
import edu.msoe.cybercheese.trinity.commands.ShooterCommands;
import edu.msoe.cybercheese.trinity.subsystems.drive.*;
import edu.msoe.cybercheese.trinity.subsystems.drive.gyro.GyroIO;
import edu.msoe.cybercheese.trinity.subsystems.drive.gyro.GyroIOCanandGyro;
import edu.msoe.cybercheese.trinity.subsystems.drive.gyro.GyroIOSim;
import edu.msoe.cybercheese.trinity.subsystems.drive.module.ModuleIO;
import edu.msoe.cybercheese.trinity.subsystems.drive.module.ModuleIOSim;
import edu.msoe.cybercheese.trinity.subsystems.drive.module.ModuleIOSpark;
import edu.msoe.cybercheese.trinity.subsystems.hopper.*;
import edu.msoe.cybercheese.trinity.subsystems.intake.*;
import edu.msoe.cybercheese.trinity.subsystems.intakeroller.IntakeRoller;
import edu.msoe.cybercheese.trinity.subsystems.intakeroller.IntakeRollerCommands;
import edu.msoe.cybercheese.trinity.subsystems.intakeroller.IntakeRollerConstants;
import edu.msoe.cybercheese.trinity.subsystems.loader.*;
import edu.msoe.cybercheese.trinity.subsystems.shooter.*;
import edu.msoe.cybercheese.trinity.subsystems.vision.*;
import edu.msoe.cybercheese.trinity.util.MathExtras;
import edu.msoe.cybercheese.trinity.util.hw.MotorConfig;
import edu.msoe.cybercheese.trinity.util.hw.MotorIO;
import edu.msoe.cybercheese.trinity.util.hw.MotorIOSim;
import edu.msoe.cybercheese.trinity.util.hw.MotorIOSpark;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.ArrayList;
import java.util.Objects;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.jspecify.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

    private static final double INTAKE_STOWED_POSITION = 0;
    private static final double INTAKE_DEPLOYED_POSITION = -2.314;
    private static final double INTAKE_MID_POSITION = (INTAKE_STOWED_POSITION + INTAKE_DEPLOYED_POSITION) / 2.0;
    private static final double[] INTAKE_POSITIONS = {
        INTAKE_STOWED_POSITION, INTAKE_MID_POSITION, INTAKE_DEPLOYED_POSITION
    };

    private final @Nullable SimulatedArena simulatedArena;
    private final @Nullable SwerveDriveSimulation driveSimulation;

    private final Drive drive;
    private final Vision vision;
    private final Shooter shooter;
    private final Intake intake;
    private final IntakeRoller intakeRoller;
    private final Hopper hopper;
    private final Loader loader;

    private int intakePositionIndex = 0;

    private final CommandXboxController controller = new CommandXboxController(0);

    private final LoggedDashboardChooser<Command> autoChooser;
    private final AutoFactory autoFactory;

    public RobotContainer() {
        CanandEventLoop.getInstance(); // starts management server for redux alchemist

        this.simulatedArena = Constants.CURRENT_MODE == Constants.Mode.SIM ? new Arena2026Rebuilt() : null;
        this.driveSimulation = this.simulatedArena != null
                ? new SwerveDriveSimulation(
                        DriveConstants.DRIVE_TRAIN_SIMULATION_CONFIG, new Pose2d(3, 3, Rotation2d.kZero))
                : null;

        final var moduleIos = new ModuleIO[DriveConstants.MODULE_DEFINITIONS.length];
        for (int i = 0; i < DriveConstants.MODULE_DEFINITIONS.length; i++) {
            moduleIos[i] = this.createModuleIo(
                    DriveConstants.MODULE_DEFINITIONS[i],
                    this.driveSimulation == null ? null : this.driveSimulation.getModules()[i]);
        }
        this.drive = new Drive(this.driveSimulation, this.createGyroIo(), moduleIos);

        final var cameras = new ArrayList<Camera>();
        for (final var cameraDef : VisionConstants.CAMERA_DEFINITIONS) {
            cameras.add(new Camera(cameraDef, this.createVisionIo(cameraDef, this.drive)));
        }
        this.vision = new Vision(this.drive, cameras);

        this.shooter = new Shooter(this.createShooterIo());
        this.intake = new Intake(this.createIntakeIo());
        this.intakeRoller = new IntakeRoller(this.createIntakeRollerIo());
        this.hopper = new Hopper(this.createHopperIo());
        this.loader = new Loader(this.createLoaderIo());

        this.autoChooser = new LoggedDashboardChooser<>("Auto Choices", new SendableChooser<>());
        if (Constants.ENABLE_SYSID) {
            this.setupSysIdAutoChooser();
        }

        this.autoFactory = new AutoFactory(
                this.drive::getPose, this.drive::setPose, this.drive::followTrajectory, true, this.drive);

        this.autoFactory.bind(
                "shoot",
                Commands.parallel(
                        ShooterCommands.runVelocity(this.shooter, 430), // this works, don't touch!
                        LoaderCommands.shootWhenReady(this.loader, this.shooter, 80),
                        HopperCommands.runVelocity(this.hopper, () -> -2700)));
        this.autoChooser.addOption(
                "Backwards Then Shoot",
                Commands.sequence(
                        Commands.run(() -> this.drive.runVelocity(new ChassisSpeeds(1.0, 0.0, 0.0)), this.drive)
                                .withTimeout(0.25),
                        Commands.runOnce(this.drive::stop, this.drive),
                        Commands.parallel(
                                ShooterCommands.runDefaultedVelocity(
                                        this.shooter, this.drive::getPose, 430, () -> false),
                                LoaderCommands.shootWhenReady(this.loader, this.shooter, 80),
                                HopperCommands.runVelocity(this.hopper, () -> -2700))));

        for (final var trajName : Choreo.availableTrajectories()) {
            System.out.println("Loading Trajectory: " + trajName);
            this.autoChooser.addOption(
                    trajName,
                    Commands.sequence(
                            this.autoFactory.resetOdometry(trajName), this.autoFactory.trajectoryCmd(trajName)));
        }

        this.configureButtonBindings();
    }

    private void setupSysIdAutoChooser() {
        this.autoChooser.addOption(
                "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        this.autoChooser.addOption(
                "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        this.autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        this.autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        this.autoChooser.addOption(
                "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        this.autoChooser.addOption(
                "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    }

    private void configureButtonBindings() {
        this.controller
                .back()
                .onTrue(Commands.runOnce(
                                () -> this.drive.setPose(
                                        new Pose2d(
                                                this.drive.getPose().getTranslation(),
                                                MathExtras.isFlipped() ? Rotation2d.kPi : Rotation2d.kZero)),
                                this.drive)
                        .ignoringDisable(true));

        this.drive.setDefaultCommand(DriveCommands.joystickDrive(
                this.drive, 
                () -> this.alterControllerInput(controller.getLeftY()),
                () -> this.alterControllerInput(controller.getLeftX()),
                () -> this.alterControllerInput(controller.getRightX()),
                false));
        this.controller
                .rightBumper()
                .whileTrue(DriveCommands.joystickDriveAtAngle(
                        this.drive,
                        () -> this.alterControllerInput(controller.getLeftY()),
                        () -> this.alterControllerInput(controller.getLeftX()),
                        () -> this.alterControllerInput(controller.getRightX()),
                        () -> Rotation2d.fromRadians(ShooterMath.absoluteHubAngle(this.drive.getPose())),
                        false));
        this.controller.x().onTrue(Commands.runOnce(this.drive::stopWithX, this.drive));

        this.hopper.setDefaultCommand(HopperCommands.runVelocity(this.hopper, () -> 0));
        this.controller.povDown().whileTrue(Commands.parallel(
            HopperCommands.runVelocity(this.hopper, () -> 2700),
            LoaderCommands.runVelocity(this.loader, -40)
        ));

        this.intake.setDefaultCommand(Commands.run(
                () -> this.intake.setLowerPosition(INTAKE_POSITIONS[this.intakePositionIndex]), this.intake));
        this.controller
                .b()
                .onTrue(Commands.runOnce(
                        () -> this.intakePositionIndex = (this.intakePositionIndex + 1) % INTAKE_POSITIONS.length));

        this.intakeRoller.setDefaultCommand(IntakeRollerCommands.runVelocity(
                this.intakeRoller,
                () -> MathUtil.clamp(-300.0 * this.controller.getLeftTriggerAxis(), -300.0, 0.0)));

        this.shooter.setDefaultCommand(ShooterCommands.runVelocity(this.shooter, 0));
        this.loader.setDefaultCommand(LoaderCommands.runVelocity(this.loader, 0));

        // this.controller.povRight().toggleOnTrue(ShooterCommands.runVelocity(this.shooter, 35));

        this.controller
                .rightTrigger()
                .whileTrue(Commands.parallel(
                        HopperCommands.shootWhenReady(
                                this.hopper,
                                this.shooter,
                                () -> this.controller.a().getAsBoolean() ? 2700 : -2700),
                        ShooterCommands.runDefaultedVelocity(
                                this.shooter,
                                this.drive::getPose,
                                430,
                                this.controller.rightBumper()), // this works, don't touch!
                        LoaderCommands.shootWhenReady(this.loader, this.shooter, 80)));

        this.controller
                .y()
                .whileTrue(Commands.parallel(
                        ShooterCommands.hailMary(this.shooter), LoaderCommands.runVelocity(this.loader, 80)));
    }

    private double alterControllerInput(double input) {
        final var deadbandedInput = MathUtil.applyDeadband(input, DriveConstants.CONTROLLER_INPUT_DEADBAND);
        double scaled = -deadbandedInput * DriveConstants.JOYSTICK_MULTIPLIER;
        if (this.controller.leftBumper().getAsBoolean()) {
            scaled *= DriveConstants.SLOW_MULTIPLIER;
        }

        return scaled;
    }

    public Command getAutonomousCommand() {
        return this.autoChooser.get();
    }

    private VisionIO createVisionIo(final VisionConstants.CameraDefinition definition, final Drive drive) {
        return switch (Constants.CURRENT_MODE) {
            case REAL -> new VisionIOPhotonVision(definition.name(), definition.transform());
            case SIM -> new VisionIOPhotonVisionSim(definition.name(), definition.transform(), drive::getPose);
            case REPLAY -> inputs -> {};
        };
    }

    private GyroIO createGyroIo() {
        return switch (Constants.CURRENT_MODE) {
            case REAL -> new GyroIOCanandGyro();
            case SIM -> new GyroIOSim(
                    Objects.requireNonNull(this.driveSimulation).getGyroSimulation());
            case REPLAY -> inputs -> {};
        };
    }

    private MotorIO createMotorIo(final int canId, final MotorConfig config) {
        return switch (Constants.CURRENT_MODE) {
            case REAL -> new MotorIOSpark(canId, config);
            case SIM -> new MotorIOSim(canId, config);
            case REPLAY -> inputs -> {};
        };
    }

    private ModuleIO createModuleIo(
            final DriveConstants.ModuleDefinition definition, final @Nullable SwerveModuleSimulation moduleSimulation) {
        return switch (Constants.CURRENT_MODE) {
            case REAL -> new ModuleIOSpark(definition);
            case SIM -> new ModuleIOSim(Objects.requireNonNull(moduleSimulation));
            case REPLAY -> inputs -> {};
        };
    }

    private MotorIO createShooterIo() {
        return this.createMotorIo(ShooterConstants.SHOOTER_MOTOR_ID, ShooterConstants.SHOOTER_MOTOR_CONFIG);
    }

    private MotorIO createIntakeIo() {
        return this.createMotorIo(IntakeConstants.LOWER_MOTOR_ID, IntakeConstants.LOWER_MOTOR_CONFIG);
    }

    private MotorIO createIntakeRollerIo() {
        return this.createMotorIo(IntakeRollerConstants.ROLLER_MOTOR_ID, IntakeRollerConstants.ROLLER_MOTOR_CONFIG);
    }

    private MotorIO createHopperIo() {
        return this.createMotorIo(HopperConstants.HOPPER_MOTOR_ID, HopperConstants.HOPPER_MOTOR_CONFIG);
    }

    private MotorIO createLoaderIo() {
        return this.createMotorIo(LoaderConstants.LOADER_MOTOR_ID, LoaderConstants.LOADER_MOTOR_CONFIG);
    }

    public void updateSimulation() {
        if (Constants.CURRENT_MODE != Constants.Mode.SIM) return;

        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput("FieldSimulation/RobotPosition", this.driveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logger.recordOutput(
                "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    }
}
