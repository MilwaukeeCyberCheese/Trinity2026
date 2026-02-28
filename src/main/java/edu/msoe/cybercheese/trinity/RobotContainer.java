package edu.msoe.cybercheese.trinity;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import com.reduxrobotics.canand.CanandEventLoop;
import edu.msoe.cybercheese.trinity.commands.DriveCommands;
import edu.msoe.cybercheese.trinity.commands.ShooterCommands;
import edu.msoe.cybercheese.trinity.subsystems.drive.*;
import edu.msoe.cybercheese.trinity.subsystems.drive.DriveConstants;
import edu.msoe.cybercheese.trinity.subsystems.drive.gyro.GyroIO;
import edu.msoe.cybercheese.trinity.subsystems.drive.gyro.GyroIOCanandGyro;
import edu.msoe.cybercheese.trinity.subsystems.drive.gyro.GyroIOSim;
import edu.msoe.cybercheese.trinity.subsystems.drive.module.ModuleIO;
import edu.msoe.cybercheese.trinity.subsystems.drive.module.ModuleIOSim;
import edu.msoe.cybercheese.trinity.subsystems.drive.module.ModuleIOSpark;
import edu.msoe.cybercheese.trinity.subsystems.hopper.*;
import edu.msoe.cybercheese.trinity.subsystems.intake.Intake;
import edu.msoe.cybercheese.trinity.subsystems.intake.IntakeCommands;
import edu.msoe.cybercheese.trinity.subsystems.intake.IntakeIO;
import edu.msoe.cybercheese.trinity.subsystems.intake.IntakeIOSpark;
import edu.msoe.cybercheese.trinity.subsystems.loader.Loader;
import edu.msoe.cybercheese.trinity.subsystems.loader.LoaderCommands;
import edu.msoe.cybercheese.trinity.subsystems.loader.LoaderIO;
import edu.msoe.cybercheese.trinity.subsystems.loader.LoaderIOSpark;
import edu.msoe.cybercheese.trinity.subsystems.shooter.*;
import edu.msoe.cybercheese.trinity.subsystems.vision.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

    private final @Nullable SimulatedArena simulatedArena;
    private final @Nullable SwerveDriveSimulation driveSimulation;

    private final Drive drive;
    private final Vision vision;
    private final Shooter shooter;
    private final Intake intake;
    private final Hopper hopper;
    private final Loader loader;

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
        this.hopper = new Hopper(this.createHopperIo());
        this.loader = new Loader(this.createLoaderIo());

        this.autoChooser = new LoggedDashboardChooser<>("Auto Choices", new SendableChooser<>());
        if (Constants.ENABLE_SYSID) {
            this.setupSysIdAutoChooser();
        }

        this.autoFactory = new AutoFactory(
                this.drive::getPose, this.drive::setPose, this.drive::followTrajectory, true, this.drive);

        for (final var trajName : Choreo.availableTrajectories()) {
            System.out.println("Loading Trajectory: " + trajName);
            this.autoChooser.addOption(trajName, this.autoFactory.trajectoryCmd(trajName));
        }

        // TODO: henry does tuning
        this.intake.setDefaultCommand(IntakeCommands.runValues(this.intake, 0, 0));
        this.hopper.setDefaultCommand(HopperCommands.runVelocity(this.hopper, 0));
        this.loader.setDefaultCommand(LoaderCommands.runVelocity(this.loader, 0));

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
        // Default command, normal field-relative drive
        this.drive.setDefaultCommand(DriveCommands.joystickDrive(
                this.drive,
                () -> controller.getLeftY() * DriveConstants.JOYSTICK_MULTIPLIER,
                () -> controller.getLeftX() * DriveConstants.JOYSTICK_MULTIPLIER,
                () -> -controller.getRightX() * DriveConstants.JOYSTICK_MULTIPLIER));

        this.shooter.setDefaultCommand(ShooterCommands.runVelocity(this.shooter, 0));

        // Lock to 0 deg when A button is held
        this.controller
                .a()
                .whileTrue(DriveCommands.joystickDriveAtAngle(
                        this.drive,
                        () -> controller.getLeftY() * DriveConstants.JOYSTICK_MULTIPLIER,
                        () -> controller.getLeftX() * DriveConstants.JOYSTICK_MULTIPLIER,
                        () -> Rotation2d.fromRadians(ShooterMath.absoluteHubAngle(this.drive.getPose()))));

        // TODO: henry needs to tune this
        this.controller.rightBumper().toggleOnTrue(IntakeCommands.runValues(this.intake, 1.5, -1000));
        this.controller.y().whileTrue(ShooterCommands.runTargetVelocity(this.shooter, this.drive::getPose));
        this.controller
                .back()
                .whileTrue(Commands.parallel(
                        LoaderCommands.runVelocity(this.loader, 500), ShooterCommands.runVelocity(this.shooter, -500)));
        this.controller.leftBumper().whileTrue(LoaderCommands.shootWhenReady(this.loader, this.shooter, 50));

        this.controller.povUp().onTrue(Commands.run(() -> intake.setLowerPosition(3), intake));
        this.controller.povDown().onTrue(Commands.run(() -> intake.setLowerPosition(-3), intake));

        this.controller.x().onTrue(Commands.runOnce(this.drive::stopWithX, this.drive));
        this.controller.povLeft().toggleOnTrue(HopperCommands.runVelocity(this.hopper, -500));

        this.controller
                .b()
                .onTrue(Commands.runOnce(
                                () -> this.drive.setPose(
                                        new Pose2d(this.drive.getPose().getTranslation(), Rotation2d.kZero)),
                                this.drive)
                        .ignoringDisable(true));
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

    private ModuleIO createModuleIo(
            final DriveConstants.ModuleDefinition definition, final @Nullable SwerveModuleSimulation moduleSimulation) {
        return switch (Constants.CURRENT_MODE) {
            case REAL -> new ModuleIOSpark(definition);
            case SIM -> new ModuleIOSim(Objects.requireNonNull(moduleSimulation));
            case REPLAY -> inputs -> {};
        };
    }

    private ShooterIO createShooterIo() {
        return switch (Constants.CURRENT_MODE) {
            case REAL -> new ShooterIOSpark();
            case SIM -> new ShooterIOSim();
            case REPLAY -> inputs -> {};
        };
    }

    private IntakeIO createIntakeIo() {
        if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
            return new IntakeIOSpark();
        }

        return inputs -> {};
    }

    private HopperIO createHopperIo() {
        return switch (Constants.CURRENT_MODE) {
            case REAL -> new HopperIOSpark();
            case SIM -> new HopperIOSim();
            case REPLAY -> inputs -> {};
        };
    }

    private LoaderIO createLoaderIo() {
        if (Constants.CURRENT_MODE == Constants.Mode.REAL) {
            return new LoaderIOSpark();
        }

        return inputs -> {};
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
